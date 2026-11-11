#include "maze.h"
#include "config.h"

// STM32 HAL Flash definitions
extern "C" {
    #include "stm32f4xx_hal.h"
}

// Direction offsets: North, East, South, West
static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {1, 0, -1, 0};

uint8_t Maze::walls[MAZE_WIDTH][MAZE_HEIGHT];
uint16_t Maze::dist[MAZE_WIDTH][MAZE_HEIGHT];
bool Maze::visited[MAZE_WIDTH][MAZE_HEIGHT];
uint8_t Maze::posX = 0;
uint8_t Maze::posY = 0;
Direction Maze::currentHeading = DIR_NORTH;
MazeTarget Maze::currentTarget = TARGET_GOAL;
bool Maze::mazeMapped = false;

void Maze::init() {
    currentTarget = TARGET_GOAL;
    resetPosition();
    // Try to load previously learned maze from Flash memory
    if (!loadFromFlash()) {
        resetWalls();
    }
    updateDistances();
}

void Maze::resetPosition() {
    posX = 0;
    posY = 0;
    currentHeading = DIR_NORTH;
    currentTarget = TARGET_GOAL;
}

void Maze::setPosition(uint8_t x, uint8_t y, Direction heading) {
    posX = x;
    posY = y;
    currentHeading = heading;
}

void Maze::setTarget(MazeTarget target) {
    currentTarget = target;
    updateDistances();
}

MazeTarget Maze::getTarget() {
    return currentTarget;
}

bool Maze::isAtTarget() {
    if (currentTarget == TARGET_START) {
        return isAtStart();
    }
    return isAtGoal();
}

void Maze::resetWalls() {
    for (uint8_t x = 0; x < MAZE_WIDTH; x++) {
        for (uint8_t y = 0; y < MAZE_HEIGHT; y++) {
            walls[x][y] = 0;
            visited[x][y] = false;
            // Outer perimeter walls
            if (y == MAZE_HEIGHT - 1) walls[x][y] |= (1 << DIR_NORTH);
            if (x == MAZE_WIDTH - 1)  walls[x][y] |= (1 << DIR_EAST);
            if (y == 0)               walls[x][y] |= (1 << DIR_SOUTH);
            if (x == 0)               walls[x][y] |= (1 << DIR_WEST);
        }
    }
    // Start cell (0, 0) East wall
    setWall(0, 0, DIR_EAST);
    visited[0][0] = true;
    mazeMapped = false;
}

void Maze::setWall(uint8_t x, uint8_t y, Direction dir) {
    if (x >= MAZE_WIDTH || y >= MAZE_HEIGHT) return;
    walls[x][y] |= (1 << dir);

    // Complementary wall in neighbor
    int8_t nx = x + DX[dir];
    int8_t ny = y + DY[dir];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
        Direction opp = (Direction)((dir + 2) % 4);
        walls[nx][ny] |= (1 << opp);
    }
}

bool Maze::hasWall(uint8_t x, uint8_t y, Direction dir) {
    if (x >= MAZE_WIDTH || y >= MAZE_HEIGHT) return true;
    return (walls[x][y] & (1 << dir)) != 0;
}

uint8_t Maze::getX() { return posX; }
uint8_t Maze::getY() { return posY; }
Direction Maze::getHeading() { return currentHeading; }

bool Maze::isAtGoal() {
    return isGoalCell(posX, posY);
}

bool Maze::isGoalCell(uint8_t x, uint8_t y) {
    return (x >= MAZE_GOAL_MIN_X && x <= MAZE_GOAL_MAX_X) &&
           (y >= MAZE_GOAL_MIN_Y && y <= MAZE_GOAL_MAX_Y);
}

bool Maze::isAtStart() {
    return (posX == 0 && posY == 0);
}

bool Maze::isMapped() {
    return mazeMapped;
}

void Maze::updateWalls(bool frontWall, bool leftWall, bool rightWall) {
    visited[posX][posY] = true;
    Direction frontDir = currentHeading;
    Direction leftDir  = (Direction)((currentHeading + 3) % 4);
    Direction rightDir = (Direction)((currentHeading + 1) % 4);

    if (frontWall) setWall(posX, posY, frontDir);
    if (leftWall)  setWall(posX, posY, leftDir);
    if (rightWall) setWall(posX, posY, rightDir);

    updateDistances();

    if (isAtGoal()) {
        mazeMapped = true;
        saveToFlash();
    }
}

void Maze::updateDistances() {
    struct Point { uint8_t x, y; };
    Point queue[MAZE_WIDTH * MAZE_HEIGHT];
    uint16_t head = 0, tail = 0;

    for (uint8_t x = 0; x < MAZE_WIDTH; x++) {
        for (uint8_t y = 0; y < MAZE_HEIGHT; y++) {
            dist[x][y] = 0xFFFF;
        }
    }

    if (currentTarget == TARGET_START) {
        // Target adalah sel Start (0, 0)
        dist[0][0] = 0;
        queue[tail++] = {0, 0};
    } else {
        // Target adalah sel Goal di labirin sesuai konfigurasi di config.h
        for (uint8_t gx = MAZE_GOAL_MIN_X; gx <= MAZE_GOAL_MAX_X; gx++) {
            for (uint8_t gy = MAZE_GOAL_MIN_Y; gy <= MAZE_GOAL_MAX_Y; gy++) {
                if (gx < MAZE_WIDTH && gy < MAZE_HEIGHT) {
                    dist[gx][gy] = 0;
                    queue[tail++] = {gx, gy};
                }
            }
        }
    }

    while (head < tail) {
        Point cur = queue[head++];
        uint16_t curDist = dist[cur.x][cur.y];

        for (uint8_t d = 0; d < 4; d++) {
            if (!hasWall(cur.x, cur.y, (Direction)d)) {
                int8_t nx = cur.x + DX[d];
                int8_t ny = cur.y + DY[d];
                if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
                    uint16_t stepCost = (currentTarget == TARGET_START && !visited[nx][ny]) ? 100 : 1;
                    if (dist[nx][ny] > curDist + stepCost) {
                        dist[nx][ny] = curDist + stepCost;
                        queue[tail++] = {(uint8_t)nx, (uint8_t)ny};
                    }
                }
            }
        }
    }
}

uint8_t Maze::getWallByte(uint8_t x, uint8_t y) {
    if (x >= MAZE_WIDTH || y >= MAZE_HEIGHT) return 0x0F;
    return walls[x][y];
}

bool Maze::hasWallAt(uint8_t x, uint8_t y, Direction dir) {
    return hasWall(x, y, dir);
}

bool Maze::isVisited(uint8_t x, uint8_t y) {
    if (x >= MAZE_WIDTH || y >= MAZE_HEIGHT) return false;
    return visited[x][y];
}

Action Maze::getNextAction() {
    if (isAtTarget()) {
        return ACTION_STOP_GOAL;
    }

    Direction bestDir = currentHeading;
    uint32_t bestCost = 0xFFFFFFFF;

    Direction testDirs[4] = {
        currentHeading,
        (Direction)((currentHeading + 1) % 4),
        (Direction)((currentHeading + 3) % 4),
        (Direction)((currentHeading + 2) % 4)
    };

    for (int i = 0; i < 4; i++) {
        Direction d = testDirs[i];
        if (!hasWall(posX, posY, d)) {
            int8_t nx = posX + DX[d];
            int8_t ny = posY + DY[d];
            if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
                if (dist[nx][ny] < 0xFFFF) {
                    uint32_t turnPenalty = 0;
                    if (d != currentHeading) {
                        int8_t diff = (int8_t)d - (int8_t)currentHeading;
                        if (diff < 0) diff += 4;
                        if (diff == 1 || diff == 3) turnPenalty = TURN_PENALTY_COST;
                        else turnPenalty = TURN_PENALTY_COST * 2;
                    }
                    uint32_t unvisitedPenalty = (currentTarget == TARGET_START && !visited[nx][ny]) ? 1000 : 0;
                    uint32_t totalCost = ((uint32_t)dist[nx][ny] * 10) + turnPenalty + unvisitedPenalty;
                    if (totalCost < bestCost) {
                        bestCost = totalCost;
                        bestDir = d;
                    }
                }
            }
        }
    }

    if (bestCost == 0xFFFFFFFF) {
        // Fallback jika tidak ada tetangga dengan jarak valid (misal jalan buntu):
        // Cari arah mana saja yang TIDAK ada dindingnya
        for (int i = 0; i < 4; i++) {
            Direction d = testDirs[i];
            if (!hasWall(posX, posY, d)) {
                int8_t nx = posX + DX[d];
                int8_t ny = posY + DY[d];
                if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
                    bestDir = d;
                    break;
                }
            }
        }
    }

    int8_t rel = (int8_t)bestDir - (int8_t)currentHeading;
    if (rel < 0) rel += 4;

    if (rel == 0) return ACTION_FORWARD;
    if (rel == 1) return ACTION_TURN_RIGHT;
    if (rel == 3) return ACTION_TURN_LEFT;
    return ACTION_TURN_AROUND;
}

void Maze::applyAction(Action action) {
    switch (action) {
        case ACTION_FORWARD: {
            int8_t nx = posX + DX[currentHeading];
            int8_t ny = posY + DY[currentHeading];
            if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
                posX = (uint8_t)nx;
                posY = (uint8_t)ny;
                visited[posX][posY] = true;
            }
            break;
        }
        case ACTION_TURN_LEFT:
            currentHeading = (Direction)((currentHeading + 3) % 4);
            break;
        case ACTION_TURN_RIGHT:
            currentHeading = (Direction)((currentHeading + 1) % 4);
            break;
        case ACTION_TURN_AROUND:
            currentHeading = (Direction)((currentHeading + 2) % 4);
            break;
        case ACTION_STOP_GOAL:
            break;
    }
}

// ==============================================================================
// FAST PATH GENERATOR & DIAGONAL SMOOTHING (JALUR TERCEPAT & GERAKAN DIAGONAL)
// Mengonversi pola zig-zag ortogonal menjadi lintasan diagonal lurus 45 derajat
// dengan kurva masuk/keluar halus (fluid slalom), mengurangi jarak tempuh ~30%.
// ==============================================================================
static uint8_t smoothDiagonalPath(const PathStep* inPath, uint8_t inCount, PathStep* outPath, uint8_t maxSteps) {
    uint8_t outCount = 0;
    uint8_t i = 0;

    while (i < inCount && outCount < maxSteps) {
        // Cek pola zig-zag: CURVE_RIGHT diikuti CURVE_LEFT
        if (i + 1 < inCount &&
            inPath[i].cmd == CMD_CURVE_RIGHT &&
            inPath[i + 1].cmd == CMD_CURVE_LEFT) {

            // Hitung jumlah pengulangan pasangan (RIGHT, LEFT) berturut-turut
            uint8_t k = 0;
            while ((i + 2 * k + 1) < inCount &&
                   inPath[i + 2 * k].cmd == CMD_CURVE_RIGHT &&
                   inPath[i + 2 * k + 1].cmd == CMD_CURVE_LEFT) {
                k++;
            }

            // Ganti rangkaian belokan zig-zag dengan lintasan diagonal
            if (outCount + 3 <= maxSteps) {
                outPath[outCount].cmd = CMD_DIAG_ENTER_RIGHT;
                outPath[outCount].cellCount = 1;
                outCount++;

                outPath[outCount].cmd = CMD_DIAG_SPRINT;
                outPath[outCount].cellCount = k;
                outCount++;

                outPath[outCount].cmd = CMD_DIAG_EXIT_LEFT;
                outPath[outCount].cellCount = 1;
                outCount++;

                i += 2 * k;
                continue;
            }
        }

        // Cek pola zig-zag: CURVE_LEFT diikuti CURVE_RIGHT
        if (i + 1 < inCount &&
            inPath[i].cmd == CMD_CURVE_LEFT &&
            inPath[i + 1].cmd == CMD_CURVE_RIGHT) {

            uint8_t k = 0;
            while ((i + 2 * k + 1) < inCount &&
                   inPath[i + 2 * k].cmd == CMD_CURVE_LEFT &&
                   inPath[i + 2 * k + 1].cmd == CMD_CURVE_RIGHT) {
                k++;
            }

            if (outCount + 3 <= maxSteps) {
                outPath[outCount].cmd = CMD_DIAG_ENTER_LEFT;
                outPath[outCount].cellCount = 1;
                outCount++;

                outPath[outCount].cmd = CMD_DIAG_SPRINT;
                outPath[outCount].cellCount = k;
                outCount++;

                outPath[outCount].cmd = CMD_DIAG_EXIT_RIGHT;
                outPath[outCount].cellCount = 1;
                outCount++;

                i += 2 * k;
                continue;
            }
        }

        // Salin langkah ortogonal standar
        outPath[outCount++] = inPath[i++];
    }

    return outCount;
}

uint8_t Maze::generateFastestPath(PathStep* pathOut, uint8_t maxSteps) {
    PathStep rawPath[64];
    uint8_t simX = 0;
    uint8_t simY = 0;
    Direction simHeading = DIR_NORTH;
    uint8_t rawStepCount = 0;
    uint8_t consecutiveFwd = 0;

    uint16_t safetyCounter = 0;
    while (!isGoalCell(simX, simY) && rawStepCount < 61) {
        // Safety: batas langkah maksimal agar tidak loop tak terhingga
        if (++safetyCounter > (MAZE_WIDTH * MAZE_HEIGHT * 2)) break;

        // Cari tetangga terbaik berdasarkan peta jarak Floodfill + Bobot Penalti Belokan
        Direction bestDir = simHeading;
        uint32_t bestCost = 0xFFFFFFFF;

        Direction testDirs[4] = {
            simHeading,
            (Direction)((simHeading + 1) % 4),
            (Direction)((simHeading + 3) % 4),
            (Direction)((simHeading + 2) % 4)
        };

        for (int i = 0; i < 4; i++) {
            Direction d = testDirs[i];
            if (!hasWall(simX, simY, d)) {
                int8_t nx = simX + DX[d];
                int8_t ny = simY + DY[d];
                if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
                    if (dist[nx][ny] < 0xFFFF) {
                        uint32_t turnPenalty = 0;
                        if (d != simHeading) {
                            int8_t diff = (int8_t)d - (int8_t)simHeading;
                            if (diff < 0) diff += 4;
                            if (diff == 1 || diff == 3) turnPenalty = TURN_PENALTY_COST;
                            else turnPenalty = TURN_PENALTY_COST * 2;
                        }
                        uint32_t totalCost = ((uint32_t)dist[nx][ny] * 10) + turnPenalty;
                        if (totalCost < bestCost) {
                            bestCost = totalCost;
                            bestDir = d;
                        }
                    }
                }
            }
        }

        // Jika tidak ada jalur valid (semua tetangga unreachable), hentikan path generation
        if (bestCost == 0xFFFFFFFF) break;

        int8_t rel = (int8_t)bestDir - (int8_t)simHeading;
        if (rel < 0) rel += 4;

        if (rel == 0) {
            consecutiveFwd++;
            int8_t nextX = simX + DX[simHeading];
            int8_t nextY = simY + DY[simHeading];
            if (nextX < 0 || nextX >= MAZE_WIDTH || nextY < 0 || nextY >= MAZE_HEIGHT) break;
            simX = (uint8_t)nextX;
            simY = (uint8_t)nextY;
        } else {
            if (consecutiveFwd > 0) {
                rawPath[rawStepCount].cmd = CMD_SPRINT;
                rawPath[rawStepCount].cellCount = consecutiveFwd;
                rawStepCount++;
                consecutiveFwd = 0;
            }

            if (rel == 1) {
                rawPath[rawStepCount].cmd = CMD_CURVE_RIGHT;
                rawPath[rawStepCount].cellCount = 1;
                rawStepCount++;
                simHeading = (Direction)((simHeading + 1) % 4);
                int8_t nxR = simX + DX[simHeading];
                int8_t nyR = simY + DY[simHeading];
                if (nxR < 0 || nxR >= MAZE_WIDTH || nyR < 0 || nyR >= MAZE_HEIGHT) break;
                simX = (uint8_t)nxR;
                simY = (uint8_t)nyR;
            } else if (rel == 3) {
                rawPath[rawStepCount].cmd = CMD_CURVE_LEFT;
                rawPath[rawStepCount].cellCount = 1;
                rawStepCount++;
                simHeading = (Direction)((simHeading + 3) % 4);
                int8_t nxL = simX + DX[simHeading];
                int8_t nyL = simY + DY[simHeading];
                if (nxL < 0 || nxL >= MAZE_WIDTH || nyL < 0 || nyL >= MAZE_HEIGHT) break;
                simX = (uint8_t)nxL;
                simY = (uint8_t)nyL;
            } else {
                rawPath[rawStepCount].cmd = CMD_PIVOT_180;
                rawPath[rawStepCount].cellCount = 1;
                rawStepCount++;
                simHeading = (Direction)((simHeading + 2) % 4);
            }
        }
    }

    if (consecutiveFwd > 0 && rawStepCount < 63) {
        rawPath[rawStepCount].cmd = CMD_SPRINT;
        rawPath[rawStepCount].cellCount = consecutiveFwd;
        rawStepCount++;
    }

    if (rawStepCount < 64) {
        rawPath[rawStepCount].cmd = CMD_FINISH;
        rawPath[rawStepCount].cellCount = 0;
        rawStepCount++;
    }

    // Terapkan diagonal smoothing (mengonversi belokan zig-zag menjadi lari diagonal mulus)
    return smoothDiagonalPath(rawPath, rawStepCount, pathOut, maxSteps);
}

// ==============================================================================
// FLASH PERSISTENCE (MEMORI FLASH SECTOR 7 UNTUK SIMPAN MAZE MAP)
// ==============================================================================
bool Maze::saveToFlash() {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_MAZE_SECTOR;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t PageError = 0;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    // Write Magic Number
    uint32_t addr = FLASH_MAZE_ADDR;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, FLASH_MAZE_MAGIC) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }
    addr += 4;

    // Write maze walls as words
    uint32_t* pData = (uint32_t*)walls;
    size_t wordsToWrite = (sizeof(walls) + 3) / 4;
    for (size_t i = 0; i < wordsToWrite; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, pData[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return true;
}

bool Maze::loadFromFlash() {
    uint32_t* flashPtr = (uint32_t*)FLASH_MAZE_ADDR;
    if (*flashPtr != FLASH_MAZE_MAGIC) {
        return false; // No valid map saved in Flash
    }

    flashPtr++; // Skip magic
    memcpy(walls, flashPtr, sizeof(walls));
    memset(visited, 1, sizeof(visited));
    mazeMapped = true;
    return true;
}

void Maze::printMaze() {
    Serial.print(F("\n--- MICROMOUSE "));
    Serial.print(MAZE_WIDTH);
    Serial.print(F("x"));
    Serial.print(MAZE_HEIGHT);
    Serial.println(F(" FLOODFILL MAP ---"));
    for (int8_t y = MAZE_HEIGHT - 1; y >= 0; y--) {
        // North walls
        for (uint8_t x = 0; x < MAZE_WIDTH; x++) {
            Serial.print(hasWall(x, y, DIR_NORTH) ? "+---" : "+   ");
        }
        Serial.println("+");

        // West/East walls and distances
        for (uint8_t x = 0; x < MAZE_WIDTH; x++) {
            Serial.print(hasWall(x, y, DIR_WEST) ? "|" : " ");
            if (x == posX && y == posY) {
                Serial.print(" * ");
            } else if (dist[x][y] == 0xFFFF) {
                Serial.print("---");
            } else {
                if (dist[x][y] < 10) Serial.print("  ");
                else if (dist[x][y] < 100) Serial.print(" ");
                Serial.print(dist[x][y]);
            }
        }
        Serial.println("|");
    }
    for (uint8_t x = 0; x < MAZE_WIDTH; x++) Serial.print("+---");
    Serial.println("+");
}

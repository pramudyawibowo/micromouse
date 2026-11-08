#include "sensors.h"
#include "config.h"

IRSensorData Sensors::rawData = {0, 0, 0, 0, 0};
IRSensorData Sensors::filteredData = {0, 0, 0, 0, 0};
uint16_t Sensors::frontThreshold = DEFAULT_WALL_FRONT_THRESHOLD;
uint16_t Sensors::sideThreshold = DEFAULT_WALL_SIDE_THRESHOLD;
uint16_t Sensors::leftNominal = 1200;
uint16_t Sensors::rightNominal = 1200;

void Sensors::init() {
    pinMode(PIN_IR_LEFT, INPUT_ANALOG);
    pinMode(PIN_IR_FRONT_L, INPUT_ANALOG);
    pinMode(PIN_IR_FRONT_C, INPUT_ANALOG);
    pinMode(PIN_IR_FRONT_R, INPUT_ANALOG);
    pinMode(PIN_IR_RIGHT, INPUT_ANALOG);

    // Initial warm-up read
    update();
    filteredData = rawData;
}

uint16_t Sensors::readAnalogSmooth(uint8_t pin, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delayMicroseconds(30);
    }
    // Inversi: TCRT5000 pada PCB ini memberi ADC rendah saat dinding dekat.
    // Dibalik agar konvensi seluruh kode konsisten: nilai TINGGI = dinding DEKAT.
    return 4095 - (uint16_t)(sum / samples);
}

void Sensors::update() {
    rawData.left        = readAnalogSmooth(PIN_IR_LEFT);
    rawData.frontLeft   = readAnalogSmooth(PIN_IR_FRONT_L);
    rawData.frontCenter = readAnalogSmooth(PIN_IR_FRONT_C);
    rawData.frontRight  = readAnalogSmooth(PIN_IR_FRONT_R);
    rawData.right       = readAnalogSmooth(PIN_IR_RIGHT);

    // Exponential Moving Average filter (alpha = 0.4)
    filteredData.left        = (uint16_t)(filteredData.left * 0.6f + rawData.left * 0.4f);
    filteredData.frontLeft   = (uint16_t)(filteredData.frontLeft * 0.6f + rawData.frontLeft * 0.4f);
    filteredData.frontCenter = (uint16_t)(filteredData.frontCenter * 0.6f + rawData.frontCenter * 0.4f);
    filteredData.frontRight  = (uint16_t)(filteredData.frontRight * 0.6f + rawData.frontRight * 0.4f);
    filteredData.right       = (uint16_t)(filteredData.right * 0.6f + rawData.right * 0.4f);
}

IRSensorData Sensors::getRaw() {
    return rawData;
}

IRSensorData Sensors::getFiltered() {
    return filteredData;
}

bool Sensors::hasWallFront() {
    // Dinding depan terdeteksi dari jarak jauh jika:
    // 1. Sensor tengah 0° (PA7) melampaui frontThreshold (~200 ADC), ATAU
    // 2. Sensor tengah mendeteksi pantulan awal (>= 140) DAN kedua sensor serong 45°
    //    juga menangkap pantulan dinding depan di atas sideThreshold.
    if (filteredData.frontCenter > frontThreshold) {
        return true;
    }
    if (filteredData.frontCenter >= 140 && 
        filteredData.frontLeft > sideThreshold && 
        filteredData.frontRight > sideThreshold) {
        return true;
    }
    return false;
}

bool Sensors::hasWallLeft() {
    return (filteredData.left > sideThreshold);
}

bool Sensors::hasWallRight() {
    return (filteredData.right > sideThreshold);
}

float Sensors::getCenteringError() {
    bool leftWall = hasWallLeft();
    bool rightWall = hasWallRight();

    if (leftWall && rightWall) {
        // Both walls present: center directly between them
        return (float)((int32_t)filteredData.left - (int32_t)filteredData.right);
    } else if (leftWall) {
        // Only left wall: maintain distance to left wall
        return (float)((int32_t)filteredData.left - (int32_t)leftNominal) * 1.5f;
    } else if (rightWall) {
        // Only right wall: maintain distance to right wall
        return (float)((int32_t)rightNominal - (int32_t)filteredData.right) * 1.5f;
    }
    // No side walls
    return 0.0f;
}

// Trigonometric Optical Angle Error:
// Mengukur kemiringan sudut robot (theta) terhadap lorong dinding secara mandiri TANPA IMU.
// Sifat trigonometri:
// (d_FL - d_FR) - (d_L - d_R) = 2 * (L_front - L_side) * sin(theta)
// Pergeseran posisi lateral (x) tereliminasi 100%, menyisakan murni nilai rotasi theta!
// Error > 0: Robot miring ke KANAN (CW) -> perlu putar ke KIRI (CCW)
// Error < 0: Robot miring ke KIRI (CCW) -> perlu putar ke KANAN (CW)
float Sensors::getAngleError() {
    if (!hasWallLeft() || !hasWallRight()) {
        return 0.0f; // Butuh kedua dinding samping agar komparasi trigonometri valid
    }

    // Selisih sensor serong depan (L_front ~ +20mm dari pusat)
    float diffFront = (float)filteredData.frontRight - (float)filteredData.frontLeft;

    // Selisih sensor samping 90 derajat (L_side ~ -58mm di dekat as roda)
    float diffSide = (float)filteredData.right - (float)filteredData.left;

    // Nilai murni kemiringan sudut
    return (diffFront - diffSide);
}

void Sensors::calibrateThresholds() {
    // Collect 50 samples in current cell
    uint32_t sumL = 0, sumR = 0, sumFC = 0;
    for (int i = 0; i < 50; i++) {
        update();
        sumL += filteredData.left;
        sumR += filteredData.right;
        sumFC += filteredData.frontCenter;
        delay(20);
    }
    leftNominal = sumL / 50;
    rightNominal = sumR / 50;

    // Validasi: jika pembacaan nominal terlalu rendah, dinding tidak terdeteksi
    // Kembalikan ke threshold default agar tidak terjadi false positive
    uint16_t calcSide = (leftNominal + rightNominal) / 4;
    uint16_t calcFront = (sumFC / 50) / 2;

    sideThreshold = (calcSide >= 80) ? calcSide : DEFAULT_WALL_SIDE_THRESHOLD;
    frontThreshold = (calcFront >= 80 && calcFront <= DEFAULT_WALL_FRONT_THRESHOLD) ? calcFront : DEFAULT_WALL_FRONT_THRESHOLD;
}

void Sensors::setThresholds(uint16_t frontThresh, uint16_t sideThresh) {
    frontThreshold = frontThresh;
    sideThreshold = sideThresh;
}

void Sensors::calibrateAmbient(uint16_t sampleCount) {
    if (sampleCount < 10) sampleCount = 10;
    uint32_t sumL = 0, sumR = 0, sumFC = 0;
    for (uint16_t i = 0; i < sampleCount; i++) {
        update();
        sumL += filteredData.left;
        sumR += filteredData.right;
        sumFC += filteredData.frontCenter;
        delay(6);
    }
    uint16_t avgL = sumL / sampleCount;
    uint16_t avgR = sumR / sampleCount;
    uint16_t avgFC = sumFC / sampleCount;

    // Hanya update nominal jika robot memang berada di samping dinding (ADC > sideThreshold)
    if (avgL > DEFAULT_WALL_SIDE_THRESHOLD) leftNominal = avgL;
    if (avgR > DEFAULT_WALL_SIDE_THRESHOLD) rightNominal = avgR;

    sideThreshold = DEFAULT_WALL_SIDE_THRESHOLD;

    // Deteksi dinding depan adaptif: Baseline ambient FC + 90 ADC (deteksi dari jarak jauh ~100-120mm)
    uint16_t dynamicFront = avgFC + 90;
    if (dynamicFront < 170) dynamicFront = 170;
    if (dynamicFront > DEFAULT_WALL_FRONT_THRESHOLD) dynamicFront = DEFAULT_WALL_FRONT_THRESHOLD;
    frontThreshold = dynamicFront;
}

uint16_t Sensors::getSideThreshold() { return sideThreshold; }
uint16_t Sensors::getFrontThreshold() { return frontThreshold; }
uint16_t Sensors::getLeftNominal() { return leftNominal; }
uint16_t Sensors::getRightNominal() { return rightNominal; }

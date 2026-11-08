#!/usr/bin/env python3
"""
Cross-platform STM32 Firmware Uploader for PlatformIO (Linux, Windows, macOS)
Supports:
1. STM32CubeProgrammer CLI (preferred, supports USB DFU bootloader & auto-run -s 0x08000000)
2. dfu-util (fallback)
"""

import sys
import os
import platform
import shutil
import subprocess

def find_stm32_programmer_cli():
    system = platform.system()
    
    # 1. Check system PATH
    cli = shutil.which("STM32_Programmer_CLI") or shutil.which("STM32_Programmer_CLI.exe")
    if cli and os.path.isfile(cli):
        return cli

    # 2. Check standard OS-specific installation paths
    candidates = []
    if system == "Linux":
        home = os.path.expanduser("~")
        candidates = [
            os.path.join(home, "STMicroelectronics", "STM32Cube", "STM32CubeProgrammer", "bin", "STM32_Programmer_CLI"),
            "/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI",
            "/opt/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI",
        ]
    elif system == "Windows":
        prog_files = os.environ.get("ProgramFiles", r"C:\Program Files")
        prog_files_x86 = os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")
        local_app_data = os.environ.get("LOCALAPPDATA", "")
        candidates = [
            os.path.join(prog_files, "STMicroelectronics", "STM32Cube", "STM32CubeProgrammer", "bin", "STM32_Programmer_CLI.exe"),
            os.path.join(prog_files_x86, "STMicroelectronics", "STM32Cube", "STM32CubeProgrammer", "bin", "STM32_Programmer_CLI.exe"),
            os.path.join(local_app_data, "Programs", "STMicroelectronics", "STM32Cube", "STM32CubeProgrammer", "bin", "STM32_Programmer_CLI.exe"),
        ]
    elif system == "Darwin":
        candidates = [
            "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI",
        ]

    for p in candidates:
        if os.path.isfile(p):
            return p

    return None

def find_dfu_util():
    cli = shutil.which("dfu-util") or shutil.which("dfu-util.exe")
    if cli and os.path.isfile(cli):
        return cli

    home = os.path.expanduser("~")
    pio_dfu = os.path.join(home, ".platformio", "packages", "tool-dfuutil", "bin", "dfu-util")
    if platform.system() == "Windows":
        pio_dfu += ".exe"
    if os.path.isfile(pio_dfu):
        return pio_dfu

    return None

def is_linux_dfu_connected():
    if platform.system() != "Linux":
        return None
    try:
        out = subprocess.check_output(["lsusb"], text=True, stderr=subprocess.DEVNULL)
        return "0483:df11" in out
    except Exception:
        return None

def main():
    if len(sys.argv) < 2:
        print("[ERROR] File firmware (.bin) tidak diberikan!")
        print("Penggunaan: python upload.py <firmware.bin>")
        sys.exit(1)

    bin_path = os.path.abspath(sys.argv[1])
    if not os.path.isfile(bin_path):
        print(f"[ERROR] File binary tidak ditemukan: {bin_path}")
        sys.exit(1)

    current_os = platform.system()
    print("=" * 65)
    print(f"🚀 Micromouse STM32 Cross-Platform Uploader ({current_os})")
    print(f"📦 Firmware: {bin_path}")
    print("=" * 65)

    cli_path = find_stm32_programmer_cli()
    dfu_path = find_dfu_util()

    # Determine port: default to usb1 (DFU). Can be overridden via env STM32_PORT
    upload_port = os.environ.get("STM32_PORT", "usb1")
    flash_addr = os.environ.get("STM32_FLASH_ADDR", "0x08000000")

    if cli_path:
        print(f"🔧 Menggunakan programmer: STM32CubeProgrammer CLI")
        print(f"   Lokasi binary: {cli_path}")
        print(f"   Target Port  : {upload_port}")
        print(f"   Alamat Flash : {flash_addr}")
        print("-" * 65)

        cmd = [
            cli_path,
            "-c", f"port={upload_port}",
            "-w", bin_path, flash_addr,
            "-v",
            "-s", flash_addr
        ]

        ret = subprocess.call(cmd)
        if ret == 0:
            print("=" * 65)
            print("✨ UPLOAD & RUN BERHASIL! Robot siap beraksi. ✨")
            print("=" * 65)
            sys.exit(0)
        else:
            print("\n" + "=" * 65)
            print("❌ GAGAL MENGUNGGAH DENGAN STM32CubeProgrammer CLI.")
            if current_os == "Linux":
                dfu_ok = is_linux_dfu_connected()
                if dfu_ok is False:
                    print("\n[DIAGNOSTIK LINUX]")
                    print("⚠️  Black Pill TIDAK terdeteksi dalam mode DFU (0483:df11) di 'lsusb'.")
                    print("👉 Cara Masuk Mode DFU:")
                    print("   1. Tahan tombol BOOT0.")
                    print("   2. Tekan lalu lepas tombol NRST (Reset).")
                    print("   3. Lepaskan tombol BOOT0.")
                    print("   4. Jalankan ulang upload.")
                else:
                    print("\n[DIAGNOSTIK LINUX]")
                    print("⚠️  Black Pill terdeteksi di USB, namun akses ditolak atau target tidak merespons.")
                    print("👉 Pastikan udev rules sudah dipasang agar user memiliki izin akses USB:")
                    print("   Jalankan perintah ini di terminal:")
                    print("     sudo ./scripts/setup_udev.sh")
                    print("   Lalu cabut & colokkan kembali kabel USB / ulangi masuk mode DFU.")
            sys.exit(ret)

    elif dfu_path:
        print(f"🔧 Menggunakan uploader fallback: dfu-util")
        print(f"   Lokasi binary: {dfu_path}")
        print(f"   Alamat Flash : {flash_addr}")
        print("-" * 65)

        cmd = [
            dfu_path,
            "-d", "0483:df11",
            "-a", "0",
            "-s", f"{flash_addr}:leave",
            "-D", bin_path
        ]

        ret = subprocess.call(cmd)
        if ret == 0:
            print("=" * 65)
            print("✨ UPLOAD DENGAN DFU-UTIL BERHASIL! ✨")
            print("=" * 65)
            sys.exit(0)
        else:
            print("\n" + "=" * 65)
            print("❌ GAGAL MENGUNGGAH DENGAN DFU-UTIL.")
            if current_os == "Linux":
                print("👉 Pastikan udev rules sudah dipasang: sudo ./scripts/setup_udev.sh")
            sys.exit(ret)

    else:
        print("\n❌ Tidak ditemukan alat upload yang kompatibel!")
        print("Silakan pasang salah satu:")
        if current_os == "Linux":
            print(" 1. STM32CubeProgrammer untuk Linux (STMicroelectronics)")
            print(" 2. dfu-util: 'sudo apt install dfu-util'")
        elif current_os == "Windows":
            print(" 1. STM32CubeProgrammer untuk Windows")
        sys.exit(1)

if __name__ == "__main__":
    main()

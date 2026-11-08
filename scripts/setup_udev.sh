#!/usr/bin/env bash
# ==============================================================================
# Setup udev rules & user permissions for STM32 / PlatformIO on Linux
# ==============================================================================

set -e

RULE_FILE="/etc/udev/rules.d/99-stm32-platformio.rules"
TARGET_USER="${SUDO_USER:-$USER}"

echo "=========================================================="
echo " Konfigurasi Udev Rules & Akses USB STM32 di Linux"
echo "=========================================================="

if [ "$EUID" -ne 0 ]; then
    echo "⚠️  Skrip ini memerlukan hak akses root untuk menyalin udev rules."
    echo "Menjalankan ulang dengan sudo..."
    exec sudo bash "$0" "$@"
fi

echo "1. Memasang udev rules ke $RULE_FILE..."
cat << 'EOF' > "$RULE_FILE"
# ----------------------------------------------------------------------
# STM32 DFU Bootloader (Black Pill F411CE / F401CC)
# ----------------------------------------------------------------------
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666", GROUP="plugdev", TAG+="uaccess"

# ----------------------------------------------------------------------
# STM32 USB CDC / Virtual COM Port (Serial Monitor)
# ----------------------------------------------------------------------
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", MODE="0666", GROUP="dialout", TAG+="uaccess"

# ----------------------------------------------------------------------
# ST-LINK v2, v2.1, v3
# ----------------------------------------------------------------------
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", GROUP="plugdev", TAG+="uaccess"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", GROUP="plugdev", TAG+="uaccess"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", GROUP="plugdev", TAG+="uaccess"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3752", MODE="0666", GROUP="plugdev", TAG+="uaccess"

# ----------------------------------------------------------------------
# General STMicroelectronics USB Devices
# ----------------------------------------------------------------------
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", MODE="0666", GROUP="plugdev", TAG+="uaccess"
EOF

chmod 644 "$RULE_FILE"

# Jika ada rule dari STM32CubeProgrammer lokal, salin juga sebagai pelengkap
CUBE_RULES_DIR="/home/$TARGET_USER/STMicroelectronics/STM32Cube/STM32CubeProgrammer/Drivers/rules"
if [ -d "$CUBE_RULES_DIR" ]; then
    echo "2. Menyalin rules tambahan dari direktori STM32CubeProgrammer..."
    cp -n "$CUBE_RULES_DIR"/*.rules /etc/udev/rules.d/ 2>/dev/null || true
fi

echo "3. Menambahkan user '$TARGET_USER' ke grup dialout dan plugdev..."
usermod -aG dialout,plugdev "$TARGET_USER" 2>/dev/null || true

echo "4. Me-reload dan men-trigger udev service..."
udevadm control --reload-rules || true
udevadm trigger || true

echo ""
echo "=========================================================="
echo "✅ Konfigurasi udev rules berhasil selesai!"
echo "=========================================================="
echo "Langkah berikutnya:"
echo "1. Cabut dan colokkan kembali kabel USB Black Pill."
echo "2. Untuk masuk mode DFU:"
echo "   - Tahan tombol BOOT0"
echo "   - Tekan dan lepas tombol NRST (Reset)"
echo "   - Lepaskan tombol BOOT0"
echo "3. Cek apakah terdeteksi: 'lsusb | grep 0483'"
echo "4. Jalankan upload: 'pio run -t upload'"
echo "=========================================================="

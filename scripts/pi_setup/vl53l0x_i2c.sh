#!/bin/bash
# VL53L0X(집게 ToF 센서)용 소프트웨어 I2C 버스 4를 켠다 (Pi에서 한 번, sudo 필요). 2026-11-01
#   sudo bash ~/polly/scripts/pi_setup/vl53l0x_i2c.sh   ->  끝나면 sudo reboot
# 배선: VCC 1번(3.3V), GND 20번, SDA 11번(GPIO17), SCL 13번(GPIO27). docs/pin_map.md
# config.txt를 고치기 전에 config.txt.bak-vl53l0x로 복사해 둔다.
# (부팅이 안 되면 SD 카드를 다른 PC에 꽂고 boot 파티션의 config.txt를 백업으로 되돌린다)
set -eu

CONFIG=/boot/firmware/config.txt
LINE="dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=17,i2c_gpio_scl=27"

if [ "$(id -u)" != 0 ]; then
  echo "sudo로 실행한다: sudo bash $0"
  exit 1
fi
if grep -q "^$LINE" "$CONFIG"; then
  echo "이미 들어 있다: $LINE"
  exit 0
fi
cp "$CONFIG" "$CONFIG.bak-vl53l0x"
printf '\n[all]\n# VL53L0X (집게 ToF) 소프트웨어 I2C 버스 4 (2026-11-01)\n%s\n' "$LINE" >> "$CONFIG"
echo "추가함: $LINE  (백업: $CONFIG.bak-vl53l0x)"
echo "이제 sudo reboot. 재부팅 뒤 확인: i2cdetect -y 4  -> 29가 보여야 한다"

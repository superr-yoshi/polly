# udev 규칙 템플릿 (부품 도착 후 값 채우기)

`/dev/ttyUSB0`, `/dev/ttyACM0` 번호는 부팅·연결 순서에 따라 바뀐다. 코드와 YAML은 아래 별칭만 쓴다.

| 별칭 | 장치 | 사용처 |
|---|---|---|
| `/dev/robot_rrc` | RRC Lite | `rrc_adapter_node` (`port`) |
| `/dev/robot_mega` | Arduino Mega 2560 | `mega_bridge_node` (`port`) |
| `/dev/robot_lidar` | RPLIDAR | sllidar_ros2 (통합 담당) |

## 1. 실제 속성 확인 (Pi에서, 장치를 하나씩 꽂으며)
```bash
ls /dev/ttyUSB* /dev/ttyACM*
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial' | head
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial' | head
```
- 정품 Mega 2560은 보통 `ttyACM*`, CH340/CP210x 계열은 `ttyUSB*`로 보인다.
- 같은 USB-Serial 칩(예: CH340)을 쓰는 장치가 둘이고 serial이 없으면 `KERNELS=="1-1.2"`처럼 **USB 포트 위치**로 구분한다.

## 2. `/etc/udev/rules.d/99-robot.rules` (실제 값으로 교체)
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="<RRC_VID>",   ATTRS{idProduct}=="<RRC_PID>",   ATTRS{serial}=="<RRC_SERIAL>",   SYMLINK+="robot_rrc",   MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="<MEGA_VID>",  ATTRS{idProduct}=="<MEGA_PID>",  ATTRS{serial}=="<MEGA_SERIAL>",  SYMLINK+="robot_mega",  MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="<LIDAR_VID>", ATTRS{idProduct}=="<LIDAR_PID>", ATTRS{serial}=="<LIDAR_SERIAL>", SYMLINK+="robot_lidar", MODE="0666"
```

## 3. 적용·확인
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/robot_*
```

## 확정값 기록 (TODO_MEASURE)
| 장치 | idVendor | idProduct | serial | 날짜 |
|---|---|---|---|---|
| RRC Lite | | | | |
| Arduino Mega | | | | |
| RPLIDAR | | | | |

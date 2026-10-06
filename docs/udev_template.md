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
- 정품 Mega 2560과 RRC Lite(CH9102)는 둘 다 `ttyACM*`로 보인다 → 번호(ACM0/ACM1)가 연결 순서에 따라 바뀌므로 반드시 serial로 구분한다.
- CH340/CP210x 계열(예전 Mega 호환보드)은 `ttyUSB*`로 보인다.
- 같은 USB-Serial 칩(예: CH340)을 쓰는 장치가 둘이고 serial이 없으면 `KERNELS=="1-1.2"`처럼 **USB 포트 위치**로 구분한다.

## 2. `/etc/udev/rules.d/99-robot.rules` (실제 값으로 교체)
```
# RRC Lite, Mega: 2026-10-06 실물에서 확인한 값. 보드를 바꾸면 serial을 다시 확인한다.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="5B22016213",           SYMLINK+="robot_rrc",  MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", ATTRS{serial}=="557363036313516052E1", SYMLINK+="robot_mega", MODE="0666"
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
| RRC Lite (USB 칩 CH9102, Linux에서 `/dev/ttyACM*`) | 1a86 | 55d4 | 5B22016213 | 2026-10-06 |
| Arduino Mega 2560 (정품, Linux에서 `/dev/ttyACM*`) | 2341 | 0042 | 557363036313516052E1 | 2026-10-06 |
| RPLIDAR | | | | |

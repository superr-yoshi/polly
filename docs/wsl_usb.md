# Windows PC에서 Mega/RRC를 WSL(Ubuntu) ROS로 테스트하기

Pi 없이 개발 PC의 WSL2 + ROS 2에서 실제 USB 장치를 쓰는 방법. (2026-09-29 Mega로 확인)

## 1회 설치 (Windows PowerShell)
```powershell
winget install --exact --id dorssel.usbipd-win
```
WSL 쪽 시리얼 권한 (1회):
```bash
sudo usermod -aG dialout $USER     # 적용하려면 Ubuntu 창을 모두 닫고 다시 연다
```

## 연결할 때마다
```powershell
usbipd list                        # Mega(CH340)는 1a86:7523, BUSID 확인 (예: 2-3)
usbipd bind --busid 2-3            # 처음 한 번만, 관리자 PowerShell
usbipd attach --wsl --busid 2-3    # USB를 다시 꽂으면 다시 실행
```
- attach 중에는 Windows(Arduino IDE, COM 포트)에서 장치가 안 보인다. 펌웨어 업로드는 `usbipd detach --busid 2-3` 후 Windows에서.
- WSL에서는 `/dev/ttyUSB0`로 보인다.

## 실행
```bash
source ~/polly/ros2_ws/install/setup.bash
ros2 run poli_hardware mega_bridge_node --ros-args -p port:=/dev/ttyUSB0
```
- `-p port:=...`는 파라미터 파일 없이 실행할 때만 먹는다. launch/params 파일을 쓰면 YAML의 노드별 값이 우선하므로
  `config/hardware.yaml`을 복사해 port를 고친 뒤 `params_file:=` 로 넘긴다.
- 다른 사람이 fake launch를 켜 두었으면 토픽이 섞인다 → `export ROS_DOMAIN_ID=77`처럼 다른 번호로 분리.

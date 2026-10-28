# Ubuntu 24.04 Pi 5에서 AI 카메라(IMX500) 쓰기

2026-10-28 Pi(`poli-desktop`)에 실제로 설치하고 확인한 과정이다. **SD카드를 새로 구우면 이 순서대로 다시 한다.**

## 왜 필요한가
- AI 카메라(Sony IMX500)는 Raspberry Pi OS 전용으로 나온다.
  Ubuntu 24.04 raspi 커널(6.8.0-1065)에는 **IMX500 드라이버가 없고**, Ubuntu의 libcamera(0.2)는 Pi 5 카메라 처리(PiSP)를 지원하지 않는다.
- 그래서 `camera_vision`(조원 B, Picamera2 사용)이 `No module named 'libcamera'` / `picamera2`로 실패했다.

## 결과 (2026-10-28)
- 커널 로그 `imx500 11-001a: Device found is imx500`, `/dev/video0~6` 생성
- `cam -l` → `1: External camera 'imx500'`, 640×480 촬영 15~30 fps
- Picamera2로 촬영 확인, `red_object_detector` 실행 가능

## 하드웨어
- 카메라 리본 케이블은 Pi 5의 **CAM/DISP 1 (J4, HDMI1 옆)**. 오버레이 기본값이 CAM/DISP 1이다.
  CAM/DISP 0(J3, 랜 포트 쪽)에 꽂으면 `config.txt`의 줄을 `dtoverlay=imx500-pi5,cam0`으로 바꾼다.
- ⚠️ 리본 케이블은 **Pi 전원을 끈 상태에서만** 꽂고 뺀다.

## 설치 순서 (Pi에서, 저장소 루트 `~/polly`)
| 단계 | 명령 | 시간 |
|---|---|---|
| 1 | `sudo bash scripts/pi_setup/imx500/install_driver.sh` → `sudo reboot` | 5~15분 |
| 2 | `sudo bash scripts/pi_setup/imx500/install_libcamera.sh --deps` | 2~5분 |
| 3 | `bash scripts/pi_setup/imx500/build_libcamera.sh` | 30~40분 |
| 4 | `sudo bash scripts/pi_setup/imx500/install_libcamera.sh` | 1분 |
| 5 | `bash scripts/pi_setup/imx500/install_picamera2.sh` | 2~3분 |

확인:
```bash
journalctl -k -b | grep imx500        # "Device found is imx500"
cam -l                                # imx500 보임
python3 -c "from picamera2 import Picamera2; print('ok')"
```

## 무엇을 바꿨나
| 항목 | 내용 |
|---|---|
| 드라이버 소스 | `raspberrypi/linux` 브랜치 `rpi-6.6.y`의 `imx500.c`, `spi-rp2040-gpio-bridge.c` (GPL-2.0) |
| `imx500.c` 수정 (`imx500-ubuntu-6.8.patch`) | 6.8 API 이름 변경 (`v4l2_subdev_get_try_*` → `v4l2_subdev_state_get_*`), `V4L2_CID_USER_IMX500_BASE`(= USER_BASE + 0x2000, libcamera와 같은 값) 정의, Ubuntu 커널이 export하지 않는 `find_cpio_data`를 `imx500_cpio.inc`로 복사 |
| 오버레이 `imx500-pi5-overlay.dts` | 카메라 클럭 노드를 `/clocks` 아래 → 루트(`/`)로 옮김. Ubuntu dtb의 `/clocks`에는 platform 장치가 안 만들어져 클럭 드라이버(`gpio-gate-clock`, 커널 내장)가 붙지 못했다 |
| 센서 펌웨어 | Raspberry Pi OS 저장소 `imx500-firmware_0.FF23+1`의 `imx500_loader.fpk`, `imx500_firmware.fpk` → `/lib/firmware/` |
| `config.txt` | `[pi5] dtoverlay=imx500-pi5` 추가 (원본 백업 `config.txt.bak-imx500`) |
| 커널 고정 | `apt-mark hold linux-image-raspi ...` — 커널이 바뀌면 드라이버를 다시 빌드해야 한다 |
| libcamera | `raspberrypi/libcamera` `v0.5.2+rpt20250903` + `libpisp v1.2.1` → `/usr/local` (Ubuntu 기본 libcamera 0.2는 그대로 둠) |
| Picamera2 | `v0.3.31` (pip `--user`). Ubuntu에 없는 `pykms`는 자리표시(`pykms.py`)로 대체 — DRM 미리보기는 쓰지 않는다 |

## 주의
- **Ubuntu 업데이트 알림의 "Install Now"를 누르지 않는다.** 커널은 고정했지만, 업데이트가 쌓이면 다시 확인이 필요하다.
- 버전 조합(libcamera 0.5.2 / libpisp 1.2.1 / Picamera2 0.3.31)은 함께 맞춘 것이다. 하나만 올리지 않는다.
- `cam_helper_imx500`의 AI 추론(네트워크 업로드) 기능은 시험하지 않았다. 지금은 일반 카메라로만 쓴다.

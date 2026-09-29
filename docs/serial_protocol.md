# POLI Serial Protocol

## 목적
Arduino Mega 2560에서 읽은 초음파 센서 및 배터리 데이터를
USB Serial을 통해 Raspberry Pi 5로 전달한다.

## 연결
- Arduino Mega 2560 → Raspberry Pi 5
- Interface: USB Serial
- Baudrate: TODO

## 기본 형식
센서 데이터는 한 줄씩 전송한다.

### Ultrasonic Range

$RNG,seq,millis,fl_mm,fr_mm,rl_mm,rr_mm*CS

예:
$RNG,15,123456,320,305,551,540*CS

### Battery

$BAT,seq,millis,millivolt*CS

예:
$BAT,16,123500,22100*CS

## 아직 미정
- Serial Baudrate: TODO
- RNG packet rate: TODO
- BAT packet rate: TODO
- Checksum / CRC rule: TODO
- Timeout / invalid range rule: TODO

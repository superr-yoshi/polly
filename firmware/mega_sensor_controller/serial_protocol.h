// serial_protocol.h
// Arduino Mega 2560 -> Raspberry Pi 5 USB Serial 프로토콜 정의
// 기준 문서: docs/serial_protocol.md
//
// 이 파일은 문서에 확정된 내용만 담는다.
// 미정인 값은 임의로 정하지 않고 TODO로 남긴다.

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

// ---------------------------------------------------------------
// 패킷 형식 (한 줄에 패킷 하나)
//
// Ultrasonic Range:
//   $RNG,seq,millis,fl_mm,fr_mm,rl_mm,rr_mm*CS
//
// Battery:
//   $BAT,seq,millis,millivolt*CS
// ---------------------------------------------------------------

// 패킷 머리말
#define PROTOCOL_HEADER_RNG "$RNG"
#define PROTOCOL_HEADER_BAT "$BAT"

// 구분 문자
#define PROTOCOL_FIELD_SEPARATOR ','
#define PROTOCOL_CHECKSUM_MARKER '*'

// ---------------------------------------------------------------
// 아직 미정 (docs/serial_protocol.md "아직 미정" 항목)
// ---------------------------------------------------------------

// TODO: Serial Baudrate
// #define PROTOCOL_BAUDRATE ...

// TODO: RNG packet rate (전송 주기, ms)
// #define PROTOCOL_RNG_PERIOD_MS ...

// TODO: BAT packet rate (전송 주기, ms)
// #define PROTOCOL_BAT_PERIOD_MS ...

// TODO: Checksum / CRC rule (CS 계산 방식)

// TODO: Timeout / invalid range rule (측정 실패 시 보낼 값)

#endif  // SERIAL_PROTOCOL_H

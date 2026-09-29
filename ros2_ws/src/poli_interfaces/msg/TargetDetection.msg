# 빨간 대상(요구조자) 탐지 결과. 보내는 쪽: 조원 B (카메라)
# 대상을 못 찾았을 때도 detected = false로 계속 보낸다.
# 자세한 설명: docs/interfaces.md

# 빨간 대상을 찾았는지
bool detected

# 화면 가운데 기준 좌우 위치, -1.0 ~ 1.0
# 음수 = 화면 왼쪽, 양수 = 화면 오른쪽
float32 x_offset

# 대상이 화면에서 차지하는 넓이 (픽셀). 가까울수록 커진다.
float32 area

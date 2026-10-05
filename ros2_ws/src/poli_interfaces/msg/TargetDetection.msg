# /vision/target : red object detection result (docs/interfaces.md 1-1)
# Published even when not found (detected = false).

bool detected      # true if a red object was found
float32 x_offset   # -1.0 ~ 1.0 from image center. negative = left, positive = right
float32 area       # object area in pixels. larger = closer

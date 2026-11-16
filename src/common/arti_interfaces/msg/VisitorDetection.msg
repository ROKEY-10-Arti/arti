# 탑캠에서 인식한 "등록된 관람객" 한 명 (인식 모듈 → arti_follow)
# 대상이 안 보이는 프레임에도 found=false로 보낸다.

std_msgs/Header header     # stamp = 탑캠 이미지 촬영 시각, frame_id = 탑캠

bool found                 # 등록된 관람객이 이 프레임에 보이는가
int32 id                   # 재식별 ID (등록할 때 정한 값)
float32 confidence         # 0~1

# YOLO 박스 (픽셀, 탑캠 원본 이미지 기준)
float32 center_x
float32 center_y
float32 width
float32 height

uint32 image_width         # 탑캠 이미지 크기 (픽셀) — 방향각 계산에 쓴다
uint32 image_height

# Single cone detection
# 锥桶颜色常量：用于区分不同边界和状态。
uint8 COLOR_UNKNOWN = 0
uint8 COLOR_BLUE    = 1
uint8 COLOR_YELLOW  = 2
uint8 COLOR_ORANGE  = 3

geometry_msgs/Point position   # 3D position in sensor/map frame
uint8 color                     # Cone color (see constants above)
float32 confidence              # Detection confidence [0.0, 1.0]

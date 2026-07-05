# Global cone map in map frame, organized by color
# 全局锥桶地图，按颜色分组，供规划和建图模块使用。
std_msgs/Header header       # frame_id: "map"

Cone[] blue_cones            # Left boundary (blue)
Cone[] yellow_cones          # Right boundary (yellow)
Cone[] orange_cones          # Start/finish line
Cone[] unknown_cones         # Unclassified

bool is_closed               # True when track loop is closed

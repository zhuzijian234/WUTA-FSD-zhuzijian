# Array of cone detections from a single source (LiDAR or camera)
# 一帧检测结果，通常由 LiDAR 或相机节点发布。
std_msgs/Header header   # frame_id: sensor frame or map frame
Cone[] cones

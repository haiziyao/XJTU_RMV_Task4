# msg/Object.msg
std_msgs/Header header      # 时间戳和坐标系
string class_name           # 物体类别
float32 x                   # 中心x坐标（像素）
float32 y                   # 中心y坐标（像素）
float32 width               # 宽度（像素）
float32 height              # 高度（像素）
float32 score               # 置信度（0~1）

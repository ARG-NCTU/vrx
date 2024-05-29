import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# 初始化ROS節點
rospy.init_node('gradient_cube')

# 創建Marker發布器
marker_pub = rospy.Publisher('/rl/visualization/region/land', Marker, queue_size=10)

# 創建Marker對象
marker = Marker()
marker.header.frame_id = "wamv/uwb/origin"
marker.header.stamp = rospy.Time.now()
marker.ns = ""
marker.id = 0
marker.type = Marker.TRIANGLE_LIST
marker.action = Marker.ADD

# 設置Marker位置和尺寸的變數
position_x = 0
position_y = 0
position_z = 1.5

scale_x = 1.5
scale_y = 1.0
scale_z = 2.0

# 設置Marker位置
marker.pose.position.x = position_x
marker.pose.position.y = position_y
marker.pose.position.z = position_z
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

# 設置Marker尺寸
marker.scale.x = 1.0  # 因為使用TRIANGLE_LIST，不需要實際設置比例因子
marker.scale.y = 1.0
marker.scale.z = 1.0

# 設置Marker頂點和顏色
points = []
colors = []

# 計算半寬、半高和半深
half_x = scale_x / 2.0
half_y = scale_y / 2.0
half_z = scale_z / 2.0

# 定義頂點座標
vertices = [
    Point(position_x + half_x, position_y + half_y, position_z + half_z),  # 上面四個頂點
    Point(position_x - half_x, position_y + half_y, position_z + half_z),
    Point(position_x - half_x, position_y - half_y, position_z + half_z),
    Point(position_x + half_x, position_y - half_y, position_z + half_z),
    Point(position_x + half_x, position_y + half_y, position_z - half_z),  # 下面四個頂點
    Point(position_x - half_x, position_y + half_y, position_z - half_z),
    Point(position_x - half_x, position_y - half_y, position_z - half_z),
    Point(position_x + half_x, position_y - half_y, position_z - half_z)
]

# 定義顏色（上面接近白色，下面綠色，半透明）
vertex_colors = [
    ColorRGBA(1.0, 1.0, 1.0, 0.8), ColorRGBA(1.0, 1.0, 1.0, 0.8), ColorRGBA(1.0, 1.0, 1.0, 0.8), ColorRGBA(1.0, 1.0, 1.0, 0.8),  # 上面四個頂點顏色
    ColorRGBA(0.0, 1.0, 0.0, 0.8), ColorRGBA(0.0, 1.0, 0.0, 0.8), ColorRGBA(0.0, 1.0, 0.0, 0.8), ColorRGBA(0.0, 1.0, 0.0, 0.8)  # 下面四個頂點顏色
]

# 定義立方體的12個三角面
triangles = [
    (0, 1, 2), (0, 2, 3),  # 上面
    (4, 5, 6), (4, 6, 7),  # 下面
    (0, 1, 5), (0, 5, 4),  # 前面
    (2, 3, 7), (2, 7, 6),  # 後面
    (1, 2, 6), (1, 6, 5),  # 左面
    (0, 3, 7), (0, 7, 4)   # 右面
]

# 添加頂點和顏色到Marker
for tri in triangles:
    points.append(vertices[tri[0]])
    colors.append(vertex_colors[tri[0]])
    points.append(vertices[tri[1]])
    colors.append(vertex_colors[tri[1]])
    points.append(vertices[tri[2]])
    colors.append(vertex_colors[tri[2]])

marker.points = points
marker.colors = colors

# 發布Marker
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    marker_pub.publish(marker)
    rate.sleep()

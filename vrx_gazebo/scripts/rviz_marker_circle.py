#!/usr/bin/env python3

import rospy
from math import cos, sin
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def draw_green_circle():
    rospy.init_node('green_circle_marker', anonymous=True)
    marker_publisher = rospy.Publisher('circle_marker', Marker, queue_size=10)
    radius = rospy.get_param("~radius", 3)
    # 創建一個Marker消息
    marker = Marker()
    marker.header.frame_id = "map"  # 請根據您的需求設置坐標系
    marker.header.stamp = rospy.Time.now()
    # marker.ns = "my_namespace"
    marker.id = 0
    marker.type = Marker.LINE_STRIP  # 使用LINE_STRIP作為形狀類型
    marker.action = Marker.ADD

    # 設置標記的位置
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # 設置標記的方向
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # 設置標記的比例
    marker.scale.x = 0.1  # 線寬度

    # 設置標記的顏色
    marker.color.a = 1.0  # 透明度
    marker.color.r = 0.0  # 紅色分量
    marker.color.g = 1.0  # 綠色分量
    marker.color.b = 0.0  # 藍色分量

    # 定義圓的輪廓點
    num_points = 360  # 可以增加或減少點的數量以控制圓的精細度
    for i in range(num_points):
        angle = 2 * 3.14159265359 * i / num_points
        point = Point()
        point.x = radius * cos(angle)  # 半徑為3.0
        point.y = radius * sin(angle)
        marker.points.append(point)

    # 設置標記的持續時間（以秒為單位）
    marker.lifetime = rospy.Duration()

    # 创建一个循环来持续绘制圆形
    rate = rospy.Rate(1)  # 每秒1次
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()  # 更新时间戳以触发rviz更新
        marker_publisher.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        draw_green_circle()
    except rospy.ROSInterruptException:
        pass

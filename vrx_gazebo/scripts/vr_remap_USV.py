#!/usr/bin/env python3
import fix_python3_path
import rospy
from sensor_msgs.msg import Joy


class VR_remap_joy:
    def __init__(self):
        self.sub_joy = rospy.Subscriber("/vr_teleop", Joy, self.cb_joy, queue_size=1)
        # self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        pub_joy_topic = rospy.get_param("~pub_joy_topic", "/wamv2/joy")
        self.pub_joy = rospy.Publisher(pub_joy_topic, Joy, queue_size=1)

        self.vr_joy = None
        self.vr_to_joy = Joy()
        self.vr_to_joy.header.frame_id = "/dev/input/js0"
        self.vr_to_joy.axes = [0.0] * 8
        self.vr_to_joy.buttons = [0] * 11

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def cb_joy(self, msg):
        self.vr_joy = msg
        print("[VR_TELEOP RECEIVED] Axes:", msg.axes, "Buttons:", msg.buttons)
        self.vr_translate_into_joy()

    def timer_callback(self, event):
        print("[TIMER CALLBACK RUNNING]")
        if self.vr_joy is None:
            print("[TIMER] No VR Joy data yet.")
            return

        self.vr_to_joy.header.stamp = rospy.Time.now()
        self.vr_translate_into_joy()

    def vr_translate_into_joy(self):
        if len(self.vr_joy.axes) > 6:
            self.vr_to_joy.axes[1] = self.vr_joy.axes[6]
        else:
            self.vr_to_joy.axes[1] = 0.0

        if len(self.vr_joy.axes) > 9:
            self.vr_to_joy.axes[3] = self.vr_joy.axes[9]
        else:
            self.vr_to_joy.axes[3] = 0.0

        self.vr_to_joy.buttons[4] = 1
        self.pub_joy.publish(self.vr_to_joy)
        # print("[JOY PUBLISHED] Axes:", self.vr_to_joy.axes, "Buttons:", self.vr_to_joy.buttons)

if __name__ == '__main__':
    rospy.init_node('vr_remap_USV')
    vr_remap = VR_remap_joy()
    rospy.spin()

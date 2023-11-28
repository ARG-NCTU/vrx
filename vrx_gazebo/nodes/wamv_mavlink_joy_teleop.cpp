#include <geometry_msgs/Twist.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <string>
#include <unordered_map>

using namespace std;

class Teleop
{
  private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub_joy_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_thrust_left_;
    ros::Publisher pub_thrust_right_;
    ros::Publisher pub_thrust_left_lateral_;
    ros::Publisher pub_thrust_right_lateral_;

    ros::ServiceClient srv_arming_;
    ros::ServiceClient srv_offboard_;
    ros::ServiceClient srv_param_set_;
    ros::ServiceClient srv_param_get_;

    ros::Timer timer_pub_;

    // ROS messages
    geometry_msgs::Twist twist;

    // ROS params
    std::unordered_map<std::string, std::unordered_map<std::string, double>> throttle_;
    double control_rate_;

    // Private variables
    bool enable_;
    std::string speed_mode_;
    std::string input_standard_;

    std::vector<double> axes_;
    std::vector<double> buttons_;
    std::unordered_map<std::string, int> axes_map_;
    std::unordered_map<std::string, int> button_map_;
    std::unordered_map<std::string, std::string> function_map_;

    void getParam();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void timerPubCallback(const ros::TimerEvent& event);
    void initMaps();
    bool updateInputStandard(const sensor_msgs::Joy::ConstPtr& msg);
    bool isPressed(std::string function, const sensor_msgs::Joy::ConstPtr& msg);
    bool isTrigger(std::string function, const sensor_msgs::Joy::ConstPtr& msg);

  public:
    Teleop(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
};

Teleop::Teleop(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private), enable_(true), speed_mode_(std::string("high")), input_standard_(std::string("XINPUT"))
{
    getParam();

    sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
    pub_twist_ = nh_private_.advertise<geometry_msgs::Twist>("twist", 10);
    pub_thrust_left_ = nh_private_.advertise<std_msgs::Float32>("thrust_left", 10);
    pub_thrust_right_ = nh_private_.advertise<std_msgs::Float32>("thrust_right", 10);
    pub_thrust_left_lateral_ = nh_private_.advertise<std_msgs::Float32>("thrust_left_lateral", 10);
    pub_thrust_right_lateral_ = nh_private_.advertise<std_msgs::Float32>("thrust_right_lateral", 10);

    srv_arming_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming", true);
    srv_offboard_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode", true);
    srv_param_set_ = nh_.serviceClient<mavros_msgs::ParamSet>("mavros/param/set", true);
    srv_param_get_ = nh_.serviceClient<mavros_msgs::ParamGet>("mavros/param/get", true);

    timer_pub_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &Teleop::timerPubCallback, this, false, false);

    axes_.resize(8);
    buttons_.resize(12);

    initMaps();

    timer_pub_.start();
    ROS_INFO_STREAM("Teleop node initialized in " << control_rate_ << " Hz");
}

void Teleop::getParam()
{
    nh_.param<double>("throttle_high_throttle", throttle_["high"]["throttle"], 1.0);
    nh_.param<double>("throttle_high_linear", throttle_["high"]["linear"], 1.0);
    nh_.param<double>("throttle_high_angular", throttle_["high"]["angular"], 1.0);

    nh_.param<double>("throttle_low_throttle", throttle_["low"]["throttle"], 0.5);
    nh_.param<double>("throttle_low_linear", throttle_["low"]["linear"], 0.5);
    nh_.param<double>("throttle_low_angular", throttle_["low"]["angular"], 0.5);

    nh_.param<double>("control_rate", control_rate_, 10.0);

    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_high_throttle: " << throttle_["high"]["throttle"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_high_linear: " << throttle_["high"]["linear"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_high_angular: " << throttle_["high"]["angular"]);

    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_low_throttle: " << throttle_["low"]["throttle"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_low_linear: " << throttle_["low"]["linear"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_low_angular: " << throttle_["low"]["angular"]);

    ROS_INFO_STREAM(ros::this_node::getName() << "control_rate: " << control_rate_);
    return;
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (updateInputStandard(msg))
    {
        return;
    }

    if (isTrigger("ARM", msg))
    {
        ROS_INFO_STREAM("A");
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        auto res = srv_arming_.call(srv);
        if (res)
        {
            ROS_INFO_STREAM("Arming");
        }
        else
        {
            ROS_ERROR_STREAM("Arming failed");
        }
    }

    if (isTrigger("OFFBOARD", msg))
    {
        ROS_INFO_STREAM("B");
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = "OFFBOARD";
        auto res = srv_offboard_.call(srv);
        if (res)
        {
            ROS_INFO_STREAM("Offboard");
        }
        else
        {
            ROS_ERROR_STREAM("Offboard failed");
        }
    }

    if (isTrigger("INITIALMAVLINK", msg))
    {
        ROS_INFO_STREAM("X");
        mavros_msgs::ParamSet srv_param_set;
        srv_param_set.request.param_id = "COM_RCL_EXCEPT";
        srv_param_set.request.value.integer = 4;
        srv_param_set_.call(srv_param_set);
        ROS_INFO_STREAM("COM_RCL_EXCEPT: " << srv_param_set.response.value.integer);

        mavros_msgs::ParamGet srv_param_get;
        srv_param_get.request.param_id = "COM_RCL_EXCEPT";
        srv_param_get_.call(srv_param_get);
        ROS_INFO_STREAM("COM_RCL_EXCEPT: " << srv_param_get.response.value.integer);

        srv_param_set.request.param_id = "COM_OBL_ACT";
        srv_param_set.request.value.integer = -1;
        srv_param_set_.call(srv_param_set);
        ROS_INFO_STREAM("COM_OBL_ACT: " << srv_param_set.response.value.integer);

        srv_param_get.request.param_id = "COM_OBL_ACT";
        srv_param_get_.call(srv_param_get);
        ROS_INFO_STREAM("COM_OBL_ACT: " << srv_param_get.response.value.integer);
    }

    std::copy(msg->axes.begin(), msg->axes.end(), axes_.begin());
    std::copy(msg->buttons.begin(), msg->buttons.end(), buttons_.begin());
    return;
}

void Teleop::timerPubCallback(const ros::TimerEvent& event)
{
    if (enable_)
    {
        if (speed_mode_ == std::string("high"))
        {
            twist.linear.x = throttle_["high"]["linear"] * axes_[axes_map_["RX"]];
            twist.linear.y = throttle_["high"]["linear"] * axes_[axes_map_["RY"]];
            twist.linear.z = throttle_["high"]["throttle"] * axes_[axes_map_["LY"]];
            twist.angular.z = throttle_["high"]["angular"] * axes_[axes_map_["LX"]];
        }
        else if (speed_mode_ == std::string("low"))
        {
            twist.linear.x = throttle_["low"]["linear"] * axes_[axes_map_["RX"]];
            twist.linear.y = throttle_["low"]["linear"] * axes_[axes_map_["RY"]];
            twist.linear.z = throttle_["low"]["throttle"] * axes_[axes_map_["LY"]];
            twist.angular.z = throttle_["low"]["angular"] * axes_[axes_map_["LX"]];
        }
        else
        {
            ROS_ERROR_STREAM("Speed mode error");
            return;
        }
        pub_twist_.publish(twist);

        std_msgs::Float32 thrust_left, thrust_right, thrust_left_lateral, thrust_right_lateral;
        thrust_left.data = axes_[axes_map_["LY"]];
        thrust_right.data = axes_[axes_map_["LY"]];
        thrust_left_lateral.data = axes_[axes_map_["RX"]];
        thrust_right_lateral.data = axes_[axes_map_["RX"]];
    }
    else
    {
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.z = 0.0;
    }
    return;
}

void Teleop::initMaps()
{
    function_map_["THROTTLE"] = std::string("LY");
    function_map_["YAW"] = std::string("LX");
    function_map_["X"] = std::string("RY");
    function_map_["Y"] = std::string("RX");
    function_map_["AUTO"] = std::string("START");
    function_map_["MANUAL"] = std::string("BACK");
    function_map_["ARM"] = std::string("A");
    function_map_["OFFBOARD"] = std::string("B");
    function_map_["INITIALMAVLINK"] = std::string("X");

    if (input_standard_ == std::string("XINPUT"))
    {
        axes_map_["LX"] = 0;
        axes_map_["LY"] = 1;
        axes_map_["LT"] = 2;
        axes_map_["RX"] = 3;
        axes_map_["RY"] = 4;
        axes_map_["RT"] = 5;
        axes_map_["DPAD_X"] = 6;
        axes_map_["DPAD_Y"] = 7;

        button_map_["A"] = 0;
        button_map_["B"] = 1;
        button_map_["X"] = 2;
        button_map_["Y"] = 3;
        button_map_["LB"] = 4;
        button_map_["RB"] = 5;
        button_map_["BACK"] = 6;
        button_map_["START"] = 7;
        button_map_["LOGO"] = 8;
        button_map_["L3"] = 9;
        button_map_["R3"] = 10;
    }
    else if (input_standard_ == std::string("DIRECTINPUT"))
    {
        axes_map_["DPAD_X"] = 0;
        axes_map_["DPAD_Y"] = 1;
        axes_map_["RX"] = 2;
        axes_map_["RY"] = 3;
        axes_map_["LX"] = 4;
        axes_map_["LY"] = 5;

        button_map_["X"] = 0;
        button_map_["A"] = 1;
        button_map_["B"] = 2;
        button_map_["Y"] = 3;
        button_map_["LB"] = 4;
        button_map_["RB"] = 5;
        button_map_["LT"] = 6;
        button_map_["RT"] = 7;
        button_map_["BACK"] = 8;
        button_map_["START"] = 9;
        button_map_["L3"] = 10;
        button_map_["R3"] = 11;
    }
    else
    {
        ROS_ERROR_STREAM("Joystick input standard error");
        return;
    }
}

bool Teleop::updateInputStandard(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->axes.size() == 6 && msg->buttons.size() == 12)
    {
        if (input_standard_ == std::string("XINPUT"))
        {
            ROS_INFO_STREAM("Joystick input standard changed to DIRECTINPUT");
            input_standard_ = std::string("DIRECTINPUT");
            initMaps();
            axes_.resize(msg->axes.size());
            buttons_.resize(msg->buttons.size());
            return true;
        }
    }
    else if (msg->axes.size() == 8 && msg->buttons.size() == 11)
    {
        if (input_standard_ == std::string("DIRECTINPUT"))
        {
            ROS_INFO_STREAM("Joystick input standard changed to XINPUT");
            input_standard_ = std::string("XINPUT");
            initMaps();
            axes_.resize(msg->axes.size());
            buttons_.resize(msg->buttons.size());
            return true;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Joystick input standard error");
        return false;
    }
    return false;
}

bool Teleop::isPressed(std::string function, const sensor_msgs::Joy::ConstPtr& msg)
{
    return msg->buttons[button_map_[function_map_[function]]];
}

bool Teleop::isTrigger(std::string function, const sensor_msgs::Joy::ConstPtr& msg)
{
    return isPressed(function, msg) && !buttons_[button_map_[function_map_[function]]];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_teleop");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Teleop teleop(nh, nh_private);
    ros::spin();
    return 0;
}

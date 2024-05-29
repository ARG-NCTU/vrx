#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <iostream>
#include <sstream>
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
    ros::Publisher pub_enable_auto_mode_success_;
    ros::Publisher pub_vertical_rod_cmd_;
    ros::Publisher pub_ready_to_track_;
    ros::Publisher pub_ready_to_landlatch_;

    ros::ServiceClient srv_arming_;
    ros::ServiceClient srv_land_;
    ros::ServiceClient srv_offboard_;
    ros::ServiceClient srv_param_set_;
    ros::ServiceClient srv_param_get_;
    ros::ServiceClient srv_kill_;

    ros::Timer timer_pub_;

    // ROS messages
    geometry_msgs::Twist twist_;
    std_msgs::Bool enable_auto_mode_;
    std_msgs::Bool ready_to_track_;
    std_msgs::Bool ready_to_landlatch_;

    // ROS params
    std::unordered_map<std::string, std::unordered_map<std::string, double>> throttle_;
    double control_rate_;

    // Private variables
    bool enable_;
    bool enable_mavlink_;
    bool vertical_rod_;
    std::string speed_mode_;
    std::string vehicle_type_;
    std::string input_standard_;

    std::vector<double> axes_;
    std::vector<double> buttons_;
    std::unordered_map<std::string, int> axes_map_;
    std::unordered_map<std::string, int> button_map_;
    std::unordered_map<std::string, std::string> function_map_;

    void getParam();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void timerPubCallback(const ros::TimerEvent &event);
    void initMaps();
    bool updateInputStandard(const sensor_msgs::Joy::ConstPtr &msg);
    bool isPressed(std::string function, const sensor_msgs::Joy::ConstPtr &msg);
    bool isTrigger(std::string function, const sensor_msgs::Joy::ConstPtr &msg);

public:
    Teleop(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
};

Teleop::Teleop(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), enable_(false), enable_mavlink_(false), vertical_rod_(false),
      speed_mode_(std::string("high")), vehicle_type_(std::string("drone")), input_standard_(std::string("XINPUT"))
{
    getParam();

    sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
    pub_twist_ = nh_private_.advertise<geometry_msgs::Twist>("twist", 10);
    pub_thrust_left_ = nh_private_.advertise<std_msgs::Float32>("thrust_left", 10);
    pub_thrust_right_ = nh_private_.advertise<std_msgs::Float32>("thrust_right", 10);
    pub_thrust_left_lateral_ = nh_private_.advertise<std_msgs::Float32>("thrust_left_lateral", 10);
    pub_thrust_right_lateral_ = nh_private_.advertise<std_msgs::Float32>("thrust_right_lateral", 10);
    pub_enable_auto_mode_success_ = nh_.advertise<std_msgs::Bool>("/enable_auto_mode_success", 10);
    pub_vertical_rod_cmd_ = nh_.advertise<std_msgs::Float64>("/drone/vertical_rod_joint_position_controller/command", 10);
    pub_ready_to_track_ = nh_.advertise<std_msgs::Bool>("/ready_to_track_success", 10);
    pub_ready_to_landlatch_ = nh_.advertise<std_msgs::Bool>("/ready_to_landlatch_success", 10);

    srv_arming_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    srv_land_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    srv_offboard_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    srv_param_set_ = nh_.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    srv_param_get_ = nh_.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    srv_kill_ = nh_.serviceClient<std_srvs::Trigger>("/drone/kill");

    timer_pub_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &Teleop::timerPubCallback, this, false, false);

    axes_.resize(8);
    buttons_.resize(12);

    initMaps();

    timer_pub_.start();
    ROS_INFO_STREAM("Teleop node initialized in " << control_rate_ << " Hz");
}

void Teleop::getParam()
{
    nh_private_.param<double>("throttle_high_throttle", throttle_["high"]["throttle"], 2.0);
    nh_private_.param<double>("throttle_high_linear", throttle_["high"]["linear"], 2.0);
    nh_private_.param<double>("throttle_high_angular", throttle_["high"]["angular"], 1.0);

    nh_private_.param<double>("throttle_low_throttle", throttle_["low"]["throttle"], 0.5);
    nh_private_.param<double>("throttle_low_linear", throttle_["low"]["linear"], 0.5);
    nh_private_.param<double>("throttle_low_angular", throttle_["low"]["angular"], 0.5);

    nh_private_.param<double>("control_rate", control_rate_, 10.0);

    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_high_throttle: " << throttle_["high"]["throttle"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_high_linear: " << throttle_["high"]["linear"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_high_angular: " << throttle_["high"]["angular"]);

    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_low_throttle: " << throttle_["low"]["throttle"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_low_linear: " << throttle_["low"]["linear"]);
    ROS_INFO_STREAM(ros::this_node::getName() << "throttle_low_angular: " << throttle_["low"]["angular"]);

    ROS_INFO_STREAM(ros::this_node::getName() << "control_rate: " << control_rate_);

    enable_auto_mode_.data = false;
    ready_to_track_.data = true;
    ready_to_landlatch_.data = false;

    return;
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    updateInputStandard(msg);

    if (isTrigger("LOGO", msg))
    {
        ROS_INFO_STREAM("LOGO");
        enable_auto_mode_.data = !enable_auto_mode_.data;
        ROS_INFO_STREAM("Track trigger: " << std::boolalpha << static_cast<bool>(enable_auto_mode_.data));
    }

    if (isTrigger("ARM_OFFBOARD", msg))
    {
        ROS_INFO_STREAM("B");

        if (!srv_param_set_.exists())
        {
            ROS_ERROR_STREAM("Param service does not exist");
            return;
        }
        if (!srv_param_get_.exists())
        {
            ROS_ERROR_STREAM("Param service does not exist");
            return;
        }
        if (!srv_arming_.exists())
        {
            ROS_ERROR_STREAM("Arming service does not exist");
            return;
        }
        if (!srv_offboard_.exists())
        {
            ROS_ERROR_STREAM("Offboard service does not exist");
            return;
        }

        // Initialize params
        mavros_msgs::ParamSet srv_param_set;
        srv_param_set.request.param_id = "COM_RCL_EXCEPT";
        srv_param_set.request.value.integer = 4;
        srv_param_set_.call(srv_param_set);

        mavros_msgs::ParamGet srv_param_get;
        srv_param_get.request.param_id = "COM_RCL_EXCEPT";
        srv_param_get_.call(srv_param_get);
        ROS_INFO_STREAM("Set COM_RCL_EXCEPT: "
                        << srv_param_set.request.value.integer
                        << ". Get COM_RCL_EXCEPT: "
                        << srv_param_get.response.value.integer);

        srv_param_set.request.param_id = "COM_OBL_ACT";
        srv_param_set.request.value.integer = -1;
        srv_param_set_.call(srv_param_set);

        srv_param_get.request.param_id = "COM_OBL_ACT";
        srv_param_get_.call(srv_param_get);
        ROS_INFO_STREAM("Set COM_OBL_ACT: "
                        << srv_param_set.request.value.integer
                        << ". Get COM_OBL_ACT: "
                        << srv_param_get.response.value.integer);

        // Arming
        mavros_msgs::CommandBool srv_command_bool;
        srv_command_bool.request.value = true;

        if (srv_arming_.call(srv_command_bool))
        {
            ROS_INFO_STREAM("Arming");
        }
        else
        {
            ROS_ERROR_STREAM("Arming failed");
        }

        // Offboard
        mavros_msgs::SetMode srv_setmode;
        srv_setmode.request.custom_mode = "OFFBOARD";
        if (srv_offboard_.call(srv_setmode))
        {
            ROS_INFO_STREAM("Offboard");
        }
        else
        {
            ROS_ERROR_STREAM("Offboard failed");
        }
    }

    if (isTrigger("LAND", msg))
    {
        ROS_INFO_STREAM("Y");
        if (srv_land_.exists())
        {
            mavros_msgs::CommandTOL srv;
            srv.request.min_pitch = 0;
            srv.request.yaw = 0;
            srv.request.latitude = 0;
            srv.request.longitude = 0;
            srv.request.altitude = 0;
            auto res = srv_land_.call(srv);
            if (res)
            {
                ROS_INFO_STREAM("Land");
            }
            else
            {
                ROS_ERROR_STREAM("Land failed");
            }
        }
        else
        {
            ROS_ERROR_STREAM("Land service does not exist");
        }
    }

    if (isTrigger("VERTICAL_ROD", msg))
    {
        ROS_INFO_STREAM("VERTICAL_ROD");
        vertical_rod_ = !vertical_rod_;
        ROS_INFO_STREAM("Vertical rod: " << std::boolalpha << static_cast<bool>(vertical_rod_));
        std_msgs::Float64 vertical_rod_cmd;
        vertical_rod_cmd.data = vertical_rod_ ? -0.2 : 0.0;
        pub_vertical_rod_cmd_.publish(vertical_rod_cmd);
    }

    if (isTrigger("AUTO", msg))
    {
        enable_auto_mode_.data = true;
        ROS_INFO_STREAM("Auto, enable: " << std::boolalpha << static_cast<bool>(enable_auto_mode_.data));
    }

    if (isTrigger("MANUAL", msg))
    {
        enable_auto_mode_.data = false;
        ROS_INFO_STREAM("Manual, enable: " << std::boolalpha << static_cast<bool>(enable_auto_mode_.data));
    }

    // if (isPressed("LB", msg))
    // {
    //     if (isTrigger("LB", msg))
    //     {
    //         ROS_INFO_STREAM("LB");
    //     }
    //     enable_mavlink_ = true;
    // }
    // else
    // {
    //     enable_mavlink_ = false;
    // }

    if (isTrigger("RB", msg))
    {
        ROS_INFO_STREAM("RB");
        if (vehicle_type_ == std::string("wamv"))
        {
            vehicle_type_ = std::string("drone");
            ROS_INFO_STREAM("Vehicle type changed to drone");
            std_msgs::Float32 thrust_left, thrust_right, thrust_left_lateral, thrust_right_lateral;
            thrust_left.data = 0.0;
            thrust_right.data = 0.0;
            thrust_left_lateral.data = 0.0;
            thrust_right_lateral.data = 0.0;
            pub_thrust_left_.publish(thrust_left);
            pub_thrust_right_.publish(thrust_right);
            pub_thrust_left_lateral_.publish(thrust_left_lateral);
            pub_thrust_right_lateral_.publish(thrust_right_lateral);
        }
        else if (vehicle_type_ == std::string("drone"))
        {
            vehicle_type_ = std::string("wamv");
            ROS_INFO_STREAM("Vehicle type changed to wamv");
            twist_.linear.x = 0.0;
            twist_.linear.y = 0.0;
            twist_.linear.z = 0.0;
            twist_.angular.z = 0.0;
            pub_twist_.publish(twist_);
        }
    }

    if (isTrigger("LB", msg))
    {
        ROS_INFO_STREAM("LB");
        enable_ = !enable_;
    }

    if (isTrigger("AUTO_TRACK", msg))
    {
        ROS_INFO_STREAM("L3");
        ready_to_track_.data = !ready_to_track_.data;
        ROS_INFO_STREAM("Ready to track: " << std::boolalpha << static_cast<bool>(ready_to_track_.data));
    }

    if (isTrigger("AUTO_LAND", msg))
    {
        ROS_INFO_STREAM("R3");
        ready_to_landlatch_.data = !ready_to_landlatch_.data;
        ROS_INFO_STREAM("Ready to landlatch: " << std::boolalpha << static_cast<bool>(ready_to_landlatch_.data));
    }

    if (isTrigger("KILL", msg))
    {
        ROS_INFO_STREAM("B");
        if (srv_kill_.exists())
        {
            std_srvs::Trigger srv;
            auto res = srv_kill_.call(srv);
            if (res)
            {
                ROS_INFO_STREAM("Kill");
            }
            else
            {
                ROS_ERROR_STREAM("Kill failed");
            }
        }
        else
        {
            ROS_ERROR_STREAM("Kill service does not exist");
        }
    }
    std::copy(msg->axes.begin(), msg->axes.end(), axes_.begin());
    std::copy(msg->buttons.begin(), msg->buttons.end(), buttons_.begin());
    return;
}

void Teleop::timerPubCallback(const ros::TimerEvent &event)
{
    if (!enable_auto_mode_.data || enable_)
    {
        if (vehicle_type_ == std::string("wamv"))
        {
            std_msgs::Float32 thrust_left, thrust_right, thrust_left_lateral, thrust_right_lateral;
            thrust_left.data = axes_[axes_map_["LY"]] * 0.3;
            thrust_right.data = axes_[axes_map_["LY"]] * 0.3;
            thrust_left_lateral.data = axes_[axes_map_["RX"]];
            thrust_right_lateral.data = -axes_[axes_map_["RX"]];
            pub_thrust_left_.publish(thrust_left);
            pub_thrust_right_.publish(thrust_right);
            pub_thrust_left_lateral_.publish(thrust_left_lateral);
            pub_thrust_right_lateral_.publish(thrust_right_lateral);
            ROS_INFO_STREAM_THROTTLE(1.0, "wamv: " << "LY: " << axes_[axes_map_["LY"]] << ", RX: " << axes_[axes_map_["RX"]]);
        }
        else if (vehicle_type_ == std::string("drone"))
        {
            twist_.linear.x = throttle_[speed_mode_]["linear"] * axes_[axes_map_["RY"]];
            twist_.linear.y = throttle_[speed_mode_]["linear"] * axes_[axes_map_["RX"]];
            twist_.linear.z = throttle_[speed_mode_]["throttle"] * axes_[axes_map_["LY"]];
            twist_.angular.z = throttle_[speed_mode_]["angular"] * axes_[axes_map_["LX"]];
            pub_twist_.publish(twist_);
            ROS_INFO_STREAM_THROTTLE(1.0, "drone: "
                                              << "  LX: " << axes_[axes_map_["LX"]]
                                              << ", LY: " << axes_[axes_map_["LY"]]
                                              << ", RX: " << axes_[axes_map_["RX"]]
                                              << ", RY: " << axes_[axes_map_["RY"]]);
        }
    }
    pub_enable_auto_mode_success_.publish(enable_auto_mode_);
    pub_ready_to_track_.publish(ready_to_track_);
    pub_ready_to_landlatch_.publish(ready_to_landlatch_);
}

void Teleop::initMaps()
{
    function_map_["THROTTLE"] = std::string("LY");
    function_map_["YAW"] = std::string("LX");
    function_map_["X"] = std::string("RY");
    function_map_["Y"] = std::string("RX");

    function_map_["AUTO"] = std::string("START");
    function_map_["MANUAL"] = std::string("BACK");

    function_map_["LAND"] = std::string("Y");
    function_map_["ARM_OFFBOARD"] = std::string("X");
    function_map_["VERTICAL_ROD"] = std::string("A");

    function_map_["AUTO_TRACK"] = std::string("L3");
    function_map_["AUTO_LAND"] = std::string("R3");

    function_map_["KILL"] = std::string("B");

    if (input_standard_ == std::string("XINPUT"))
    {
        axes_map_.clear();
        axes_map_["LX"] = 0;
        axes_map_["LY"] = 1;
        axes_map_["LT"] = 2;
        axes_map_["RX"] = 3;
        axes_map_["RY"] = 4;
        axes_map_["RT"] = 5;
        axes_map_["DPAD_X"] = 6;
        axes_map_["DPAD_Y"] = 7;

        button_map_.clear();
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
        axes_map_.clear();
        axes_map_["LX"] = 0;
        axes_map_["LY"] = 1;
        axes_map_["RX"] = 2;
        axes_map_["RY"] = 3;
        axes_map_["DPAD_X"] = 4;
        axes_map_["DPAD_Y"] = 5;

        button_map_.clear();
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

bool Teleop::updateInputStandard(const sensor_msgs::Joy::ConstPtr &msg)
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

bool Teleop::isPressed(const std::string function,
                       const sensor_msgs::Joy::ConstPtr &msg)
{
    // if founciton is found in function_map_
    if (function_map_.find(function) != function_map_.end())
    {
        return msg->buttons[button_map_[function_map_[function]]];
    }
    else
    {
        return msg->buttons[button_map_[function]];
    }
}

bool Teleop::isTrigger(const std::string function, const sensor_msgs::Joy::ConstPtr &msg)
{
    if (function_map_.find(function) != function_map_.end())
    {
        return isPressed(function, msg) && !buttons_[button_map_[function_map_[function]]];
    }
    else
    {
        return isPressed(function, msg) && !buttons_[button_map_[function]];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_teleop");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Teleop teleop(nh, nh_private);
    ros::spin();
    return 0;
}

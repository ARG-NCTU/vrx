#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <string>

class ModelStateToPoseStamped
{
  public:
    ModelStateToPoseStamped() : ready(false)
    {
        ros::NodeHandle nh("");
        ros::NodeHandle nh_private("~");
        nh_private.param("model_name", model_name, std::string("wamv"));
        nh_private.param("pose_topic_suffix", pose_topic_suffix, std::string("/pose_stamped"));
        nh_private.param("publish_rate", publish_rate, 100);

        std::string pose_topic = "/gazebo";
        if (!model_name.empty() && model_name[0] == '/')
        {
            pose_topic += model_name;
        }
        else
        {
            pose_topic += "/" + model_name;
        }
        if (!pose_topic_suffix.empty() && pose_topic_suffix[0] == '/')
        {
            pose_topic += pose_topic_suffix;
        }
        else
        {
            pose_topic += "/" + pose_topic_suffix;
        }

        pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
        model_sub = nh.subscribe("/gazebo/model_states", 10, &ModelStateToPoseStamped::modelCallback, this);

        pose_msg.header.frame_id = "map";  // or whatever the fixed frame is in your context

        timer = nh.createTimer(ros::Duration(1.0 / publish_rate), &ModelStateToPoseStamped::publishPose, this);
    }

    void modelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        try
        {
            auto model_index = std::find(msg->name.begin(), msg->name.end(), model_name) - msg->name.begin();
            if (model_index >= msg->name.size())
            {
                ROS_ERROR_STREAM_THROTTLE(1, "Model name " << model_name << " not found in the model states.");
                return;
            }

            ROS_INFO_STREAM_THROTTLE(1, "Model name: " << msg->name[model_index]);
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose = msg->pose[model_index];
            ready = true;
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Error: " << e.what());
        }
    }

    void publishPose(const ros::TimerEvent&)
    {
        if (!ready)
        {
            return;
        }
        ROS_INFO_STREAM_THROTTLE(1, "Publishing pose for " << model_name);
        pose_publisher.publish(pose_msg);
    }

  private:
    ros::Publisher pose_publisher;
    ros::Subscriber model_sub;
    ros::Timer timer;
    geometry_msgs::PoseStamped pose_msg;
    std::string model_name;
    std::string pose_topic_suffix;
    int publish_rate;
    bool ready;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_state_to_pose_stamped");
    ModelStateToPoseStamped mstp;
    ros::spin();
    return 0;
}

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

class ModelStateToTf
{
  public:
    ModelStateToTf()
    {
        ros::NodeHandle nh("");
        ros::NodeHandle nh_private("~");
        nh_private.param("model_name", model_name, std::string("wamv"));
        nh_private.param("model_tf_suffix", model_tf_suffix, std::string("base_link"));
        nh_private.param("broadcast_rate", broadcast_rate, 100);

        if (!model_tf_suffix.empty() && model_tf_suffix[0] == '/')
        {
            child_frame_id = model_name + model_tf_suffix;
        }
        else
        {
            child_frame_id = model_name + "/" + model_tf_suffix;
        }

        // child_frame_id = model_name + model_tf_suffix if no leading slash else model_name + "/" + model_tf_suffix
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();
        model_sub = nh_private.subscribe("/gazebo/model_states", 1000, &ModelStateToTf::modelCallback, this);

        tf_msg.header.frame_id = "map";  // or the name of your world frame

        timer = nh_private.createTimer(ros::Duration(1.0 / broadcast_rate), &ModelStateToTf::broadcastTf, this);
    }

    void modelCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        try
        {
            auto model_index = std::find(msg->name.begin(), msg->name.end(), model_name) - msg->name.begin();
            if (model_index >= msg->name.size())
            {
                ROS_WARN_STREAM_THROTTLE(1, "Model name " << model_name << " not found in model states message");
                return;
            }

            ROS_INFO_STREAM_THROTTLE(1, "Model name: " << msg->name[model_index]);

            tf_msg.header.stamp = ros::Time::now();
            tf_msg.child_frame_id = child_frame_id;
            tf_msg.transform.translation.x = msg->pose[model_index].position.x;
            tf_msg.transform.translation.y = msg->pose[model_index].position.y;
            tf_msg.transform.translation.z = msg->pose[model_index].position.z;
            tf_msg.transform.rotation = msg->pose[model_index].orientation;
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Error: " << e.what());
        }
    }

    void broadcastTf(const ros::TimerEvent&)
    {
        if (tf_msg.child_frame_id.empty())
        {
            ROS_WARN_STREAM("Not ready yet");
            return;
        }
        ROS_INFO_STREAM_THROTTLE(1, "Broadcasting transform from " << tf_msg.header.frame_id << " to " << tf_msg.child_frame_id);
        tf_broadcaster->sendTransform(tf_msg);
    }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    ros::Subscriber model_sub;
    ros::Timer timer;
    geometry_msgs::TransformStamped tf_msg;
    std::string model_name;
    std::string model_tf_suffix;
    std::string child_frame_id;
    int broadcast_rate;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_state_to_tf");
    ModelStateToTf mstf;
    ros::spin();
    return 0;
}

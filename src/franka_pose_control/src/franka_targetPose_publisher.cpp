#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class TargetPosePublisher:public rclcpp::Node{
    public:
        TargetPosePublisher():Node("Franka_Commd_Publish_node_cpp"){
            RCLCPP_INFO(this->get_logger(),"Command publisher created!");
        
            subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("sigma1/pose",10,
                                        std::bind(&TargetPosePublisher::do_cb,this,std::placeholders::_1));

            publisher_ = create_publisher<geometry_msgs::msg::Pose>("target_pose", 10);

            timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                               std::bind(&TargetPosePublisher::publishTargetPose, this));

        }
    private:
        void publishTargetPose()
        {
            // Publish the joint state message
            auto target_pose_msg = geometry_msgs::msg::Pose();
            target_pose_msg.position.x =  0.28  + pose_received_.pose.position.x;
            target_pose_msg.position.y =  0.00  + pose_received_.pose.position.y;
            target_pose_msg.position.z =  0.60  + pose_received_.pose.position.z;

            target_pose_msg.orientation.w = pose_received_.pose.orientation.w;
            target_pose_msg.orientation.x = pose_received_.pose.orientation.x;
            target_pose_msg.orientation.y = pose_received_.pose.orientation.y;
            target_pose_msg.orientation.z = pose_received_.pose.orientation.z;

            RCLCPP_INFO(this->get_logger(),"Target Pose Publishered!!");
            publisher_->publish(target_pose_msg);
        };

        void do_cb(const geometry_msgs::msg::PoseStamped &msg){
            RCLCPP_INFO(this->get_logger(),"Position received: (x: %.2f, y: %.2f, z: %.2f)",
                            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
            pose_received_ = msg;
        };

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::PoseStamped pose_received_;

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto franka_listen_node = std::make_shared<TargetPosePublisher>();
    rclcpp::spin(franka_listen_node);
    rclcpp::shutdown();
    return 0;
}
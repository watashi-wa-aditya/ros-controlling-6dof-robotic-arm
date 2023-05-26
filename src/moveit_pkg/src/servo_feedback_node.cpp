#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16MultiArray.h>
#include <tf/transform_broadcaster.h>

std_msgs::Int16MultiArray feedback_msg;
ros::Publisher pub;

void feedbackCallback(const std_msgs::Int16MultiArray& feedback_msg)
{
    // Create a JointState message and populate it with the servo feedback data
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.push_back("base_joint");
    joint_state.name.push_back("elbow");
    joint_state.name.push_back("shoulder");
    joint_state.position.push_back(feedback_msg.data[0]);
    joint_state.position.push_back(feedback_msg.data[1]);
    joint_state.position.push_back(feedback_msg.data[2]);



    // Publish the JointState message
    pub.publish(joint_state);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // Define the initial position and orientation of each link relative to its parent link
    tf2::Quaternion waist_orientation(0, 0, 0, 1);
    tf2::Quaternion arm1_orientation(0, 0, 0, 1); // Quaternion representing the orientation of arm1 relative to waist
    tf2::Quaternion arm2_orientation(0, 0, 0, 1); // Quaternion representing the orientation of arm2 relative to arm1
    tf2::Quaternion gripper_orientation(0, 0, 0, 1); // Quaternion representing the orientation of gripper relative to arm2
    

    // Transform the waist link and its child links
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base";
    transformStamped.child_frame_id = "waist";
    transformStamped.transform.translation.x = -0.00345;
    transformStamped.transform.translation.y = -1e-05;
    transformStamped.transform.translation.z = 0.04449;
    waist_orientation.setRPY(0, 0, feedback_msg.data[0] * M_PI/180.0);
    transformStamped.transform.rotation.x = waist_orientation.x();
    transformStamped.transform.rotation.y = waist_orientation.y();
    transformStamped.transform.rotation.z = waist_orientation.z();
    transformStamped.transform.rotation.w = waist_orientation.w();
    br.sendTransform(transformStamped);

    // Transform the arm1 link and its child links
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base";
    transformStamped.child_frame_id = "arm1";
    transformStamped.transform.translation.x = -0.003895641743;
    transformStamped.transform.translation.y = 0.013691;
    transformStamped.transform.translation.z = 0.03521;
    arm1_orientation.setRPY(feedback_msg.data[1] * M_PI/180.0, 0, 0);
    transformStamped.transform.rotation.x = arm1_orientation.x();
    transformStamped.transform.rotation.y = arm1_orientation.y();
    transformStamped.transform.rotation.z = arm1_orientation.z();
    transformStamped.transform.rotation.w = arm1_orientation.w();
    br.sendTransform(transformStamped);

    // Transform the arm2 link and its child links
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "arm1";
    transformStamped.child_frame_id = "arm2";
    transformStamped.transform.translation.x = -0.000068;
    transformStamped.transform.translation.y = 0.116813;
    transformStamped.transform.translation.z = -0.00006769848355;
    arm2_orientation.setRPY(0, 0, feedback_msg.data[2] * M_PI/180.0);
    transformStamped.transform.rotation.x = arm2_orientation.x();
    transformStamped.transform.rotation.y = arm2_orientation.y();
    transformStamped.transform.rotation.z = arm2_orientation.z();
    transformStamped.transform.rotation.w = arm2_orientation.w();
    br.sendTransform(transformStamped);
}


int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "base_joint_servo_feedback_node");
    ros::NodeHandle nh;

    // Create a publisher for the JointState message
    pub = nh.advertise<sensor_msgs::JointState>("joint_values", 1);

    // Subscribe to the servo feedback topic
    ros::Subscriber sub = nh.subscribe("servo_feedback", 1, feedbackCallback);

    // Spin the node
    ros::spin();

    return 0;
}

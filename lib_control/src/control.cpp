#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>

float location_x[13] =     {+2.24f, +1.91f, +0.51f, -1.69f, -1.69f, -3.09f, -3.23f, -3.06f, -2.15f, -0.18f, +1.47f, +2.24f, +2.24f};
float location_y[13] =     {+1.66f, -0.37f, +0.24f, +0.24f, +0.24f, +0.31f, +2.38f, +4.58f, +5.00f, +5.00f, +5.00f, +1.66f, +1.66f};
float location_theta[13] = {-1.57f, +1.57f, +1.57f, +1.57f, +1.57f, +0.43f, +0.00f, -0.34f, -1.16f, -1.57f, -1.57f, +3.17f, -1.57f};

float grip_pose_x[13] = {+2.24f, +1.91f, +0.51f, -1.69f, -1.69f, -3.09f, -3.23f, -3.06f, -2.15f, -0.18f, +1.47f, +2.24f, +2.24f};
float grip_pose_y[13] = {+1.66f, -0.37f, +0.24f, +0.24f, +0.24f, +0.31f, +2.38f, +4.58f, +5.00f, +5.00f, +5.00f, +1.66f, +1.66f};
float grip_pose_z[13] = {-1.57f, +1.57f, +1.57f, +1.57f, +1.57f, +0.43f, +0.00f, -0.34f, -1.16f, -1.57f, -1.57f, +3.17f, -1.57f};

ros::Publisher pub, pubGrip, pubMain;

void stateCallback(const std_msgs::String& msg);
void navCallback(const geometry_msgs::Pose2D& msg);
void gripCallback(const std_msgs::Bool& msg);
float goal_x = 0, goal_y = 0, goal_theta = 0, grip_x = 0, grip_y = 0, grip_z = 0;
bool isGrip = false;

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;

    // Init Publisher and Subcriber
    pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
    pubGrip = nh.advertise<geometry_msgs::Point>("/goal_point", 100);
    pubMain = nh.advertise<std_msgs::String>("/Main_state", 100);
    ros::Subscriber state_subscribe = nh.subscribe("/Main_state", 1000, stateCallback);
    ros::Subscriber nav_subscribe = nh.subscribe("/Nav_point", 1000, navCallback);
    ros::Subscriber grip_subscribe = nh.subscribe("/grip_state", 1000, gripCallback);

    ros::spin();
    return 0;
}

void stateCallback(const std_msgs::String& msg)
{
    if (msg.data == "Navigation") {
        geometry_msgs::PoseStamped goal;
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        tf::Quaternion orientation = tf::createQuaternionFromYaw(goal_theta);
        quaternionTFToMsg(orientation, goal.pose.orientation);
        pub.publish(goal);
    }
    else if (msg.data == "Gripper") {
        if (!isGrip) {
            geometry_msgs::Point grip;
            grip.x = grip_x;
            grip.y = grip_y;
            grip.z = grip_z;
            pubGrip.publish(grip);
        }
        else {
            geometry_msgs::Point grip;
            grip.x = 0.0;
            grip.y = 0.0;
            grip.z = 0.0;
            pubGrip.publish(grip);
        }
    }

}

void navCallback(const geometry_msgs::Pose2D& msg)
{
    goal_x = location_x[int(msg.x)];
    goal_y = location_y[int(msg.x)];
    goal_theta = location_theta[int(msg.x)];
    grip_x = grip_pose_x[int(msg.y)];
    grip_y = grip_pose_y[int(msg.y)];
    grip_z = grip_pose_z[int(msg.y)];
}

void gripCallback(const std_msgs::Bool& msg)
{
    isGrip = msg.data;
    if (msg.data) {
        geometry_msgs::PoseStamped goal;
        goal.pose.position.x = 0.0;
        goal.pose.position.y = 0.0;
        tf::Quaternion orientation = tf::createQuaternionFromYaw(0.0);
        quaternionTFToMsg(orientation, goal.pose.orientation);
        pub.publish(goal);
    }
    else {
        std_msgs::String main;
        main.data = "Done";
        pubMain.publish(main);
    }
}
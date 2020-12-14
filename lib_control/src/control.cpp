#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

float location_x[5] =     {-0.50f, -1.50f, -2.50f, -3.50f, -4.50f};
float location_y[5] =     {+4.00f, +4.00f, +4.00f, +4.00f, +4.00f};
float location_theta[5] = {-1.57f, -1.57f, -1.57f, -1.57f, -1.57f};

float grip_pose_x[5] = {-0.50f, -1.50f, -2.50f, -3.50f, -4.50f};
float grip_pose_y[5] = {+4.00f, +4.00f, +4.00f, +4.00f, +4.00f};
float grip_pose_z[3] = {+0.30f, +0.60f, +0.90f};

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
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        tf::Quaternion orientation = tf::createQuaternionFromYaw(goal_theta);
        quaternionTFToMsg(orientation, goal.pose.orientation);
        pub.publish(goal);
    }
    else if (msg.data == "Gripper") {
        if (isGrip = false) {
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
    grip_x = grip_pose_x[int(msg.x)];
    grip_y = grip_pose_y[int(msg.x)];
    grip_z = grip_pose_z[int(msg.y)];
}

void gripCallback(const std_msgs::Bool& msg)
{
    isGrip = msg.data;
    if (isGrip) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = -2.0;
        goal.pose.position.y = 1.0;
        tf::Quaternion orientation = tf::createQuaternionFromYaw(goal_theta);
        quaternionTFToMsg(orientation, goal.pose.orientation);
        pub.publish(goal);
    }
    else {
        std_msgs::String main;
        main.data = "Done";
        pubMain.publish(main);
    }
}
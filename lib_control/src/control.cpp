#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
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
void gripCallback(const std_msgs::Int16& msg);
float goal_x = 0, goal_y = 0, goal_theta = 0, grip_x = 0, grip_y = 0, grip_z = 0;
int isGrip = 0;
std_msgs::Int16 grip;

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;

    // Init Publisher and Subcriber
    pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);
    pubGrip = nh.advertise<std_msgs::Int16>("/state", 100);
    pubMain = nh.advertise<std_msgs::String>("/Main_state", 100);
    ros::Subscriber state_subscribe = nh.subscribe("/Main_state", 1000, stateCallback);
    ros::Subscriber nav_subscribe = nh.subscribe("/Nav_point", 1000, navCallback);
    ros::Subscriber grip_subscribe = nh.subscribe("/state", 1000, gripCallback);

    ros::Rate r(10.0);
    while(nh.ok()){
        grip.data = 0;
        pubGrip.publish(grip);
        ros::spinOnce();
        r.sleep();
    }

    // ros::spin();
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
        if (isGrip == 0) {
            std_msgs::Int16 grip;
            grip.data = 1;
            pubGrip.publish(grip);
        }
        else if (isGrip == 1) {
            std_msgs::String main;
            main.data = "Done";
            pubMain.publish(main);
            std_msgs::Int16 grip;
            grip.data = 3;
            pubGrip.publish(grip);
        }
    }
}

void navCallback(const geometry_msgs::Pose2D& msg)
{
    goal_x = location_x[int(msg.x-1)];
    goal_y = location_y[int(msg.x-1)];
    goal_theta = location_theta[int(msg.x-1)];
    grip_x = grip_pose_x[int(msg.x-1)];
    grip_y = grip_pose_y[int(msg.x-1)];
    grip_z = grip_pose_z[int(msg.y-1)];
}

void gripCallback(const std_msgs::Int16& msg)
{
    isGrip = msg.data;
    if (isGrip == 1) {
        sleep(5);
        std_msgs::Int16 grip;
        grip.data = 2;
        pubGrip.publish(grip);
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = -2.0;
        goal.pose.position.y = 1.0;
        tf::Quaternion orientation = tf::createQuaternionFromYaw(goal_theta);
        quaternionTFToMsg(orientation, goal.pose.orientation);
        pub.publish(goal);
    }
}
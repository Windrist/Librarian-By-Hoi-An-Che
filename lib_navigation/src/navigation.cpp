#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#define PI 3.141592654
#define V_x 0.12
#define V_z 1.2
#define distance_to_stop 0.2
#define MAX_SPEED 1216
#define MAX_W 19
#define R 0.065
#define L 0.15

ros::ServiceClient cli_makePlan;
nav_msgs::GetPlan srv;
ros::Publisher pub, pubMain;

float odom_x = 0, odom_y = 0, goal_x = 0, goal_y = 0, goal_theta = 0, path_x = 0, path_y = 0, theta = 0;
bool getGoal = false;
bool isFirstPath = true;

void goalCallback(const geometry_msgs::PoseStamped& msg);
void odomCallback(const nav_msgs::Odometry& msg);
void set_robot_move(float x, float z);
void navigation_process();

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh;

    // Init Publisher and Subcriber
    cli_makePlan = nh.serviceClient<nav_msgs::GetPlan>("/global_plan/planner/make_plan");
    pub = nh.advertise<geometry_msgs::Twist>("/omni_mobile/cmd_vel", 100);
    pubMain = nh.advertise<std_msgs::String>("/Main_state", 100);
    ros::Subscriber goal_subscribe = nh.subscribe("/move_base_simple/goal", 1000, goalCallback);
    ros::Subscriber odom_subscribe = nh.subscribe("/amcl", 1000, odomCallback);
    
    ros::Rate r(10);
    while (ros::ok()) {

        // If get Goal from Move_base
        if (getGoal)
            navigation_process();

        ros::spinOnce();
		r.sleep();
    }
    ros::spin();
    return 0;
}

void goalCallback(const geometry_msgs::PoseStamped& msg)
{
    goal_x = msg.pose.position.x;
    goal_y = msg.pose.position.y;
    goal_theta = tf::getYaw(msg.pose.orientation);
    getGoal = true;
}

void odomCallback(const nav_msgs::Odometry& msg)
{
    odom_x = msg.pose.pose.position.x;
    odom_y = msg.pose.pose.position.y;
    theta = tf::getYaw(msg.pose.pose.orientation);
}

void set_robot_move(float x, float z)
{
    // Init Variable
    geometry_msgs::Twist twist;
    twist.linear.x = x;
	twist.angular.z = z;
    
    // Publish Speed
    pub.publish(twist);
}

void navigation_process()
{
	nav_msgs::Odometry odom;
    float x = 0, z = 0;

    // Set Start Pose
    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = odom_x;
    srv.request.start.pose.position.y = odom_y;
    // Set Goal Pose
    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;

    // Call Path
    if (cli_makePlan.call(srv)) {
        path_x = srv.response.plan.poses[10].pose.position.x;
        path_y = srv.response.plan.poses[10].pose.position.y;
    }

    // Get Distance
    float dis = sqrt(pow(path_y - odom_y, 2)+pow(path_x - odom_x, 2));
    float phi = atan2(path_y - odom_y, path_x - odom_x) - theta;
    if (phi > PI)
        phi -= 2 * PI;
    else if (phi < -PI)
        phi += 2 * PI;

    // Set Velocity to Head to Goal
    if ((fabs(phi) > 0.1) && (isFirstPath == 1) || (fabs(phi) > 1.5)) {
        if (phi < 0)
            z = -V_z;
        else
            z = V_z;
    }
    else if(dis > 0.0001) {
        isFirstPath = 0;
        x = V_x;
        z =  V_z * phi;
    }
    else {
        x = V_x;
    }

	// Stop
    if (sqrt(pow(goal_y - odom_y, 2) + pow(goal_x - odom_x, 2)) < distance_to_stop) {
        if (fabs(theta - goal_theta) > 0.1) {
            x = 0;
            z = V_z;
        }
        else {
            x = 0;
            z = 0;
            isFirstPath = true;
            getGoal = false;
            std_msgs::String main_msg;
            main_msg.data = "Gripper";
            pubMain.publish(main_msg);
        }
    }
    set_robot_move(x, z);
}
#include <ros/ros.h>    
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#define PI 3.14159265

class OmniMobile
{
public:
    float pos_x, pos_y, pos_th;
    float lin_vel, ang_vel;
    tf::TransformBroadcaster odom_br;
    ros::Time last_command_time;
    ros::Subscriber vel_sub;

    OmniMobile(float x, float y, float theta)
    {
        pos_x = x;
        pos_y = y;
        pos_th = theta;
    }

    void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        last_command_time = ros::Time::now();
        lin_vel = msg->linear.x;
        ang_vel = msg->angular.z;
    }

    void update(float dt)
    {
        if(ros::Time::now() - last_command_time > ros::Duration(0.1))
        {
            lin_vel = 0.0;
            ang_vel = 0.0;
        }
        
        pos_th += ang_vel*dt;
        pos_th -= 2*PI * std::floor((pos_th+PI)/(2*PI));
        pos_x += std::cos(pos_th)*lin_vel*dt;
        pos_y += -std::sin(pos_th)*lin_vel*dt;

        geometry_msgs::TransformStamped odom_tr;
        odom_tr.header.stamp = ros::Time::now();
        odom_tr.header.frame_id = "map";
        odom_tr.child_frame_id = "base_link";

        odom_tr.transform.translation.x = pos_x;
        odom_tr.transform.translation.y = -pos_y;
        odom_tr.transform.translation.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos_th);
        odom_tr.transform.rotation = odom_quat;

        odom_br.sendTransform(odom_tr);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_robot");
    ros::NodeHandle nh;

    OmniMobile robot(0.0, 0.0, 0.0);
    robot.vel_sub = nh.subscribe("/omni_mobile/cmd_vel", 10, &OmniMobile::velocityCallback, &robot);
    
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("/omni_mobile/pose", 100);

    ros::Time curr_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    ros::Rate r(100);
    while(ros::ok())
    {
        curr_time = ros::Time::now();
        
        float dt = curr_time.toSec() - last_time.toSec();
        robot.update(dt);

        geometry_msgs::Pose2D pose;
        pose.x = robot.pos_x;
        pose.y = robot.pos_y;
        pose.theta = robot.pos_th;
        pose_pub.publish(pose);

        last_time = curr_time;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
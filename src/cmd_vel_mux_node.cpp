#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

std::string user_vel_topic = "/joy_vel";
std::string shared_dwa_topic = "/shared_dwa/cmd_vel";
std::string local_planner_topic = "/local_planner/cmd_vel";
std::string cmd_vel_topic = "/cmd_vel";

geometry_msgs::Twist user_vel, shared_dwa_vel, local_planner_vel;

void userVelCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    user_vel = *msg;
}

void sharedVelCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    shared_dwa_vel = *msg;
}

void localPlannerVelCB(const geometry_msgs::Twist::ConstPtr &msg)
{
    local_planner_vel = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_mux");
    ros::NodeHandle nh("~");

    //Read topic parameters
    nh.getParam("user_vel_topic", user_vel_topic);
    nh.getParam("shared_dwa_topic", shared_dwa_topic);    
    nh.getParam("local_planner_topic", local_planner_topic);

    ros::Subscriber user_vel_sub = nh.subscribe(user_vel_topic, 1, &userVelCB);
    ros::Subscriber shared_vel_sub = nh.subscribe(shared_dwa_topic, 1, &sharedVelCB);
    ros::Subscriber local_planner_vel_sub = nh.subscribe(local_planner_topic, 1, &localPlannerVelCB);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

    ROS_INFO("Command velocity mux node started");

    ros::Rate r(20);
    while(ros::ok())
    {
        ros::spinOnce();

        //If user velocity is 0, switch mux to output local_planner's velocity 
        if(user_vel.linear.x == 0 && user_vel.angular.z == 0)
            cmd_vel_pub.publish(local_planner_vel);

        //Otherwise output shared_dwa's velocity
        else
            cmd_vel_pub.publish(shared_dwa_vel);

        r.sleep();
    }
}
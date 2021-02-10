#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

std::string user_vel_topic = "/joy_vel";
std::string shared_dwa_topic = "/shared_dwa/cmd_vel";
std::string local_planner_topic = "/local_planner/cmd_vel";
std::string cmd_vel_topic = "/cmd_vel";
double rate = 20;
double blend_time = 1.0;
bool autonomous_enabled = true;

ros::Publisher autonomous_enabled_pub;

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

void enableAutonomousCB(const std_msgs::Bool::ConstPtr &msg)
{
    if(msg->data != autonomous_enabled)
    {
        autonomous_enabled = msg->data;
        autonomous_enabled_pub.publish(*msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_mux");
    ros::NodeHandle nh("~");

    //Read topic parameters
    nh.getParam("user_vel_topic", user_vel_topic);
    nh.getParam("shared_dwa_topic", shared_dwa_topic);
    nh.getParam("local_planner_topic", local_planner_topic);
    nh.getParam("rate", rate);
    nh.getParam("blend_time", blend_time);
    nh.getParam("autonomous_enabled", autonomous_enabled);

    ros::Subscriber user_vel_sub = nh.subscribe(user_vel_topic, 1, &userVelCB);
    ros::Subscriber shared_vel_sub = nh.subscribe(shared_dwa_topic, 1, &sharedVelCB);
    ros::Subscriber local_planner_vel_sub = nh.subscribe(local_planner_topic, 1, &localPlannerVelCB);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    ros::Subscriber enable_autonomous_sub = nh.subscribe("enable_autonomous", 1, &enableAutonomousCB);
    autonomous_enabled_pub = nh.advertise<std_msgs::Bool>("autonomous_enabled", 1, true);

    ROS_INFO("Command velocity mux node started");

    ros::Rate r(rate);

    int channel = 0, prev_channel = 0;

    //Blending counters
    int curr_step = 0, max_steps = rate * blend_time;
    while(ros::ok())
    {
        ros::spinOnce();

        //If velocity was coming from autonomous to shared_dwa, blend it
        //channel 0 is autonomous velocity
        //Rounding to set a band before checking for 0
        double v_cmd = round(user_vel.linear.x * 10) / 10.0;
        double w_cmd = round( user_vel.angular.z * 10) / 10.0;
        if(v_cmd == 0 && w_cmd == 0 && autonomous_enabled)
        {
            channel = 0;

            //There was a change in channel from shared_dwa to autonomous
            if(prev_channel != channel)
                prev_channel = channel;

            cmd_vel_pub.publish(local_planner_vel);
        }

        else
        {
            channel = 1;
            //There was a change from autonomous to shared_dwa
            if(prev_channel != channel)
            {
                prev_channel = channel;
                curr_step = 0;
            }

            if(curr_step < max_steps)
            {
                geometry_msgs::Twist temp_twist;
                temp_twist.linear.x = (double) curr_step / max_steps * shared_dwa_vel.linear.x + (double)(max_steps - curr_step) / max_steps * local_planner_vel.linear.x;
                temp_twist.angular.z = (double) curr_step / max_steps * shared_dwa_vel.angular.z + (double)(max_steps - curr_step) / max_steps * local_planner_vel.angular.z;
                curr_step++;
                cmd_vel_pub.publish(temp_twist);
            }

            else
                cmd_vel_pub.publish(shared_dwa_vel);
        }

        r.sleep();
    }
}

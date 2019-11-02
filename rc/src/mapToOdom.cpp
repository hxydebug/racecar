#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"

class ET
{
    public:
        ET();
        void path_cb(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);

        tf::TransformListener tf_listener;
        
        ros::Publisher plan_pub;
        ros::Subscriber plan_sub;

        ros::Publisher goal_pub;
        ros::Subscriber goal_sub;
};



ET::ET(){
    ros::NodeHandle n;    
    plan_sub = n.subscribe("/move_base/TebLocalPlannerROS/local_plan", 1, &ET::path_cb, this);
    plan_pub = n.advertise<nav_msgs::Path>("/move_base/TebLocalPlannerROS/local_plan_odom", 10);

    goal_sub = n.subscribe("/move_base_simple/goal", 1, &ET::goalCB, this);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal_odom", 10);
}


void ET::path_cb(const nav_msgs::Path::ConstPtr& pathMsg)
{   nav_msgs::Path map_path;
    nav_msgs::Path odom_path;

    map_path = *pathMsg;
    for(int i =0; i< map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;
            

            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                odom_path.poses.push_back(odom_path_pose);
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("path tf %s",ex.what());
                // ros::Duration(1.0).sleep();
            }
        }
    plan_pub.publish(odom_path);
}



void ET::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        goal_pub.publish(odom_goal);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("goal tf %s",ex.what());
        // ros::Duration(1.0).sleep();
    }

}


int main(int argc, char **argv){
    ros::init(argc, argv, "nav_planToOdom");
    ET et;
    ros::spin();
    return 0;
}

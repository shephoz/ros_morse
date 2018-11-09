#include <math.h>
#include <algorithm>
#include <vector>
using namespace std;

#include <ros/ros.h>
#include <tf2/utils.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include <geometry_msgs/PoseStamped.h>

#include "vec.cpp"
#include "phase_manager.cpp"

class Test1
{
    public:
        Test1(PhaseManager phase);
        ~Test1();
        void run();
    private:
        ros::NodeHandle nh_;
        double loop_rate_;

        // To receive the odometry
        ros::Subscriber odom_sub_;
        ros::Subscriber goal_sub_;
        void odomCallback(const nav_msgs::Odometry& msg);
        void goalCallback(const geometry_msgs::PoseStamped& msg);
        nav_msgs::Odometry odom_;
        geometry_msgs::PoseStamped goal_;

        // To send the velocity command
        ros::Publisher cmd_vel_pub_;

        double gapof_angular(double gap_yaw, double curr_yaw);
        double make_angular(double gap_yaw, double curr_yaw);

        void decideVel();
        void setVel(double linear, double angular);

        PhaseManager phase_;


};

Test1::Test1(PhaseManager phase)
{
    // Get parameter from the parameter server
    ros::NodeHandle pnh_ = ros::NodeHandle();
    pnh_.param("loop_rate", loop_rate_, 5.0);

    // Register the callback for odometry
    odom_sub_ = nh_.subscribe("xbot/odom", 10, &Test1::odomCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &Test1::goalCallback, this);

    // Inform about the velocity publishing
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("xbot/cmd_vel", 10, false);

    phase_ = phase;


}

Test1::~Test1()
{
}

void Test1::odomCallback(const nav_msgs::Odometry& msg)
{
    //ROS_INFO("Callback called");
    // Copy the data
    // Avoid long work in the callback
    this->odom_ = msg;
}

void Test1::goalCallback(const geometry_msgs::PoseStamped& msg)
{
    // Copy the data
    // Avoid long work in the callback
    this->goal_ = msg;
    phase_.addPhase( 3.0, 3.0, -1*M_PI);
    //phase_.addPhase(  4.0,   2.0, -1*M_PI);
    //phase_.addPhase(msg.pose.position.x, msg.pose.position.y, -1*M_PI);

}

void Test1::run()
{
    geometry_msgs::Quaternion orientation;


    // Set the loop frequency
    ros::Rate loop_rate(this->loop_rate_);

    // Start the loop
    while(ros::ok())
    {
        // Check for callbacks
        ros::spinOnce();

        //time = odom_.header.stamp.toSec();
        orientation = odom_.pose.pose.orientation;

        // Do something with the callback data
        // ROS_INFO("odom orientation: [%f %f %f %f]", orientation.x, orientation.y, orientation.z, orientation.w);
        // ROS_INFO("odom yaw: %f", tf2::getYaw(orientation));
        ROS_INFO("odom position: [%f %f]", odom_.pose.pose.position.x, odom_.pose.pose.position.y);
        ROS_INFO("goal position: [%f %f]", goal_.pose.position.x, goal_.pose.position.y);

        decideVel();

        // Wait if necessary to respect the loop rate
        loop_rate.sleep();
    }
}

void Test1::decideVel(){
    if(phase_.isEnd()){
        setVel(0.0,0.0);
        ROS_INFO("end");
        return;
    }

    double near_rotation = M_PI / 12;
    double near_hypot    = 0.1;
    double max_speed = 2.0;
    double min_speed = 0.5;

    double goal_x_   = phase_.getGoalX();
    double goal_y_   = phase_.getGoalY();
    double goal_yaw_ = phase_.getGoalYaw();

    double curr_x = odom_.pose.pose.position.x;
    double curr_y = odom_.pose.pose.position.y;
    double curr_yaw = tf2::getYaw(odom_.pose.pose.orientation);

    double gap_x = goal_x_ - curr_x;
    double gap_y = goal_y_ - curr_y;
    double gap_yaw = atan2(gap_y, gap_x);

    ROS_INFO("yaw : %f + %f", curr_yaw, gap_yaw);
    ROS_INFO("phase %d : (%f,%f) => (%f,%f)", phase_.num(), curr_x, curr_y, goal_x_, goal_y_);

    double linear;
    if(phase_.isAttituding()){
        ROS_INFO("attituding");
        if(abs(gapof_angular(gap_yaw,curr_yaw)) < near_rotation) phase_.endAttituding();
        linear  = 0.0;
    }else{
        linear  = hypot(gap_x,gap_y);
        linear  = std::min(max_speed, linear);
        linear  = std::max(min_speed, linear);
    }
    setVel(linear, make_angular(gap_yaw,curr_yaw));

    if(hypot(gap_x,gap_y) < near_hypot){
        phase_.nextPhase();
    }
}



void Test1::setVel(double linear, double angular){
    geometry_msgs::Twist cmd_vel_;
    cmd_vel_.linear.x  = linear ;
    cmd_vel_.angular.z = angular; //0.0 * M_PI / 180.0;
    cmd_vel_pub_.publish(cmd_vel_);
    ROS_INFO("sent vel : linear[%f], angular[%f]", linear, angular);

}

double Test1::gapof_angular(double gap_yaw, double curr_yaw){
    double angular = gap_yaw - curr_yaw;
    if(angular > M_PI)      angular -= 2 * M_PI;
    if(angular < M_PI * -1) angular += 2 * M_PI;
    return angular;
}

double Test1::make_angular(double gap_yaw, double curr_yaw){
    double max_rotation = M_PI / 3;
    double angular = gapof_angular(gap_yaw,curr_yaw);
    if(angular > max_rotation)      angular = max_rotation;
    if(angular < max_rotation * -1) angular = max_rotation * -1;
    return angular;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_1");

    PhaseManager pm;
    Test1 test1(pm);

    test1.run();

    return 0;
}

// https://github.com/shouhei182/seimitsu_enshu/blob/main/enshu1/src/myrobot.h
// clang-format off
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <turtlebot3_msgs/SensorState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

const double BURGER_MAX_LIN_VEL = 0.22;
const double BURGER_MAX_ANG_VEL = 2.84;

class MyRobot
{
    private:
        ros::Publisher pub_vel_;
        ros::Subscriber sensor_state_;
        ros::Subscriber odom_;
        int ls_ = 0;
        int rs_ = 0;
        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;
        ros::Time t_start_;

    double clip(double n, double abs_max) 
    {
        abs_max = std::abs(abs_max);
        return std::max(-abs_max, std::min(n, abs_max));
    }

    void setSensor(const turtlebot3_msgs::SensorState sensor)
    {
        ls_ = int(sensor.sonar)/1000;
        rs_ = int(sensor.sonar)%1000;
    }

    void setOdom(const nav_msgs::Odometry odom)
    {
        x_ = odom.pose.pose.position.x;
        y_ = odom.pose.pose.position.y;
        theta_ = odom.pose.pose.orientation.z;
    }

    public:
        MyRobot()
        {
            ros::NodeHandle n;
            pub_vel_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            sensor_state_ = n.subscribe("/sensor_state", 1, &MyRobot::setSensor, this);
            odom_ = n.subscribe("/odom", 1, &MyRobot::setOdom, this);
            t_start_= ros::Time::now();
        }

        int get_ls()
        {
            return ls_;
        }

        int get_rs()
        {
            return rs_;
        }

        double get_x()
        {
            return x_;
        }

        double get_y()
        {
            return y_;
        }

        double get_theta()
        {
            return theta_;
        }

        void move(double v, double omega)
        {
            geometry_msgs::Twist msg;
            msg.linear.x = clip(v, BURGER_MAX_LIN_VEL);
            msg.angular.z = clip(omega, BURGER_MAX_ANG_VEL);
            pub_vel_.publish(msg);
        }

        void stop()
        {
            geometry_msgs::Twist msg;
            msg.linear.x = 0.;
            msg.angular.z = 0.;
            pub_vel_.publish(msg);
        }

        void wait(double t)
        {
            ros::Duration(t).sleep();
        }

        double get_time()
        {
            return (ros::Time::now() - t_start_).toSec();
        }

        void end()
        {
            ros::shutdown();
        }
};
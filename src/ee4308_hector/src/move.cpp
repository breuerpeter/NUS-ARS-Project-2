#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <signal.h>
#include "common.hpp"
#define NaN std::numeric_limits<double>::quiet_NaN()

ros::ServiceClient en_mtrs;
void disable_motors(int sig)
{
    ROS_INFO(" HMOVE : Disabling motors...");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv); 
}

double target_x = NaN, target_y = NaN, target_z = NaN;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_x = msg->point.x;
    target_y = msg->point.y;
    target_z = msg->point.z;
}

double x = NaN, y = NaN, z = NaN, a = NaN;
void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    x = p.x;
    y = p.y;
    z = p.z;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    a = atan2(siny_cosp, cosy_cosp);
}

bool rotate = false;
void cbRotate(const std_msgs::Bool::ConstPtr &msg)
{
    rotate = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" HMOVE : Param enable_move not found, set to true");
    if (!enable_move)
        return 0;
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" HMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" HMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double Kp_z;
    if (!nh.param("Kp_z", Kp_z, 1.0))
        ROS_WARN(" HMOVE : Param Kp_z not found, set to 1.0");
    double Ki_z;
    if (!nh.param("Ki_z", Ki_z, 0.0))
        ROS_WARN(" HMOVE : Param Ki_z not found, set to 0");
    double Kd_z;
    if (!nh.param("Kd_z", Kd_z, 0.0))
        ROS_WARN(" HMOVE : Param Kd_z not found, set to 0");
    double yaw_rate;
    if (!nh.param("yaw_rate", yaw_rate, 0.5))
        ROS_WARN(" HMOVE : Param yaw_rate not found, set to 0.5");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 2.0))
        ROS_WARN(" HMOVE : Param max_lin_vel not found, set to 2");
    double max_z_vel;
    if (!nh.param("max_z_vel", max_z_vel, 0.5))
        ROS_WARN(" HMOVE : Param max_z_vel not found, set to 0.5");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" HMOVE : Param move_iter_rate not found, set to 25");
    
    // --------- Enable Motors ----------
    ROS_INFO(" HMOVE : Enabling motors...");
    en_mtrs = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = true;
    if (en_mtrs.call(en_mtrs_srv))
        ROS_INFO(" HMOVE : Motors enabled!");
    else
        ROS_WARN(" HMOVE : Cannot enable motors!");
    signal(SIGINT, disable_motors);

    // --------- Subscribers ----------
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);
    ros::Subscriber sub_rotate = nh.subscribe("rotate", 1, &cbRotate);

    // --------- Publishers ----------
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // --------- Wait for Topics ----------
    ROS_INFO(" HMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && (std::isnan(target_x) || std::isnan(x))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce(); // update the topics

    // --------- Begin Controller ----------
    ROS_INFO(" HMOVE : ===== BEGIN =====");
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic
    double cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z, cmd_lin_vel_a;
    double dt;
    double prev_time = ros::Time::now().toSec();

    // --------- Open file for logging PID ----------
    std::ofstream data_file_z, data_file_lin;
    // replace with local file directory
    // file path as follows "/home/<user>/<project folder>/PID_z_data.txt"
    data_file_z.open("/home/ubuntu2004/EE4308_Project2/PID_z_data.txt");
    if (data_file_z.fail()) ROS_INFO("H PID Z: File Open Failed");
    else ROS_INFO("H PID Z: File Open Success");
    // file path as follows "/home/<user>/<project folder>/PID_lin_data.txt"
    data_file_lin.open("/home/ubuntu2004/EE4308_Project2/PID_lin_data.txt");
    if (data_file_lin.fail()) ROS_INFO("H PID Lin: File Open Failed");
    else ROS_INFO("H PID Lin: File Open Success");

////////////////// DECLARE VARIABLES HERE //////////////////
double z_error = target_z - z;
double z_error_prev = z_error, z_error_sum = 0, z_error_derr = 0;
double x_error = 0; // initial target is directly above
double x_error_prev = x_error, x_error_sum = 0, x_error_derr = 0;
double y_error = 0; // initial target is directly above 
double y_error_prev = y_error, y_error_sum = 0, y_error_derr = 0;
// hector is initially stationary so there is no control signal at t=0
cmd_lin_vel_x = 0, cmd_lin_vel_y = 0, cmd_lin_vel_z = 0, cmd_lin_vel_a = 0;
double ux = 0, ux_prev = 0;
double uy = 0, uy_prev = 0;
double uz = 0, uz_prev = 0;
// variables for experimentation
// rise time variables
double rise_time_z_start = 0, rise_time_y_start = 0, rise_time_x_start = 0;
double rise_time_z_end = 0, rise_time_y_end = 0, rise_time_x_end = 0;
bool rt_z_start = false, rt_z_end = false;
bool rt_x_start = false, rt_x_end = false;
bool rt_y_start = false, rt_y_end = false;
double rise_time_z = 0, rise_time_y = 0, rise_time_x = 0;
// max overshoot variables
double max_os_z = -10, max_os_y = -10, max_os_x = -10;
bool start_lin_expts = false;
// positions of hector at start of linear experiments 
double pos_x_start = 0, pos_y_start = 0; 

    // main loop
    while (ros::ok() && nh.param("run", true))
    {
        // update all topics
        ros::spinOnce();

        dt = ros::Time::now().toSec() - prev_time;
        if (dt == 0) // ros doesn't tick the time fast enough
            continue;
        prev_time += dt;

        // publish speeds
        msg_cmd.linear.x = cmd_lin_vel_x;
        msg_cmd.linear.y = cmd_lin_vel_y;
        msg_cmd.linear.z = cmd_lin_vel_z;
        msg_cmd.angular.z = cmd_lin_vel_a;
        pub_cmd.publish(msg_cmd);

        //// IMPLEMENT /////
        
        // PID controller for z axis
        z_error = target_z - z;
        z_error_sum += z_error;
        z_error_derr = (z_error - z_error_prev) / dt;
        z_error_prev = z_error;
        uz = Kp_z*z_error + Ki_z*z_error_sum + Kd_z*z_error_derr;
        // Acceleration
        double acc_z = (uz - uz_prev) / dt;

        // Velocity for z axis
        cmd_lin_vel_z = uz_prev + acc_z*dt;
        cmd_lin_vel_z = sat(cmd_lin_vel_z, max_z_vel); // saturate
        uz_prev =  cmd_lin_vel_z;

        // Start/stop rotating upon command
        cmd_lin_vel_a = rotate?yaw_rate:0;

        // Transform coordinates of target to robot frame
        // Assume negligible effect of pitch and roll
        /*
            Transformation matrix to convert from world frame to robot frame
            [ cos(a)  sin(a)]   [target_x - x]   [error_x]
            [-sin(a)  cos(a)] * [target_y - y] = [error_y]
        */
        
        // Coordinates of target in robot frame
        double target_x_rbt, target_y_rbt,relative_x, relative_y;
        relative_x = target_x - x;
        relative_y = target_y - y;
        target_x_rbt = cos(a)*relative_x + sin(a)*relative_y;
        target_y_rbt = -sin(a)*relative_x + cos(a)*relative_y;
        //ROS_INFO("target_x_rbt = %3.3lf, target_y_rbt = %3.3lf", target_x_rbt, target_y_rbt);

        // Compute relative size of x_error and y_error to determine saturation velocity for x and y
        // relative size of x_error compared to y_error
        double k, sat_vel_x, sat_vel_y;
        if (target_y_rbt != 0) {
            k = abs(target_x_rbt/target_y_rbt);
            /*  total velocity need to be <= 2 m/s so sqrt(vel_x*vel_x + vel_y*vel_y) <= max_lin_vel
            if error_x / error_y = k, then vel_x / vel_y = k
            combining the 2 equations, we get max vel_y = max_lin_vel / sqrt(k*k+1) and max vel_x = k * vel_y 
            */
            sat_vel_y = max_lin_vel / sqrt(k*k + 1);
            sat_vel_x = k * sat_vel_y;
        } else {
            // prevent division by 0 error when y error (target_y_rbt) is 0
            sat_vel_y = 0;
            sat_vel_x = max_lin_vel;
        }

        // PID controller for x axis
        x_error = target_x_rbt;
        x_error_sum += x_error;
        x_error_derr = (x_error - x_error_prev) / dt;
        x_error_prev = x_error;
        ux = Kp_lin*x_error + Ki_lin*x_error_sum + Kd_lin*x_error_derr;
        // Acceleration
        double acc_x = (ux - ux_prev) / dt;

        // Velocity for x axis
        cmd_lin_vel_x = ux_prev + acc_x*dt;
        cmd_lin_vel_x = sat(cmd_lin_vel_x, sat_vel_x); // saturate
        ux_prev =  cmd_lin_vel_x;

        // PID controller for y axis
        y_error = target_y_rbt;
        y_error_sum += y_error;
        y_error_derr = (y_error - y_error_prev) / dt;
        y_error_prev = y_error;
        uy = Kp_lin*y_error + Ki_lin*y_error_sum + Kd_lin*y_error_derr;
        // Acceleration
        double acc_y = (uy - uy_prev) / dt;

        // Velocity for y axis
        cmd_lin_vel_y = uy_prev + acc_y*dt;
        cmd_lin_vel_y = sat(cmd_lin_vel_y, sat_vel_y); // saturate
        uy_prev =  cmd_lin_vel_y;

        // verbose
        if (verbose)
        {
            // ROS_INFO(" HMOVE : Target(%6.3f, %6.3f, %6.3f) FV(%6.3f) VX(%6.3f) VY(%6.3f) VZ(%7.3f)", target_x, target_y, target_z, cmd_lin_vel, cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z);
            
            // --------- Experiments for z ----------
            // Compute the rise time for z axis
            if (z >= 0.20 && !rt_z_start) {
                rise_time_z_start = ros::Time::now().toSec();
                rt_z_start = true;
            } else if (z >= 1.80 && !rt_z_end) {
                rise_time_z_end = ros::Time::now().toSec();
                rt_z_end = true;
                rise_time_z = rise_time_z_end - rise_time_z_start;
            }
            // Compute max overshoot for z axis
            if (z>=2) {
                max_os_z = std::max(max_os_z, z-2);
            }
            double ess_z = 2-z;
            ROS_INFO("Steady state erorr z = %3.3lf", ess_z); 
            ROS_INFO("Maximum overshoot z = %3.3lf", max_os_z);
            ROS_INFO("Rise Time z = %3.3lf", rise_time_z);

            // write data to file
            // time vs z
            data_file_z << prev_time << "\t" << z << std::endl;

            // --------- Experiments for linear ----------

            ROS_INFO("Orientation: %3.3f", a);
            ROS_INFO("Z = %3.3lf", z);
            // Start linear experiments only when hector has reached certain height
            if (z>=2 && !start_lin_expts) {
                start_lin_expts = true;
                ROS_INFO("Linear Experiments Start");
                pos_x_start = x;
                pos_y_start = y;
            }
            if (start_lin_expts) {
                // Compute the rise time for x axis
                if ((x >= 0.20 + pos_x_start) && !rt_x_start) {
                    rise_time_x_start = ros::Time::now().toSec();
                    rt_x_start = true;
                } else if ((x >= 1.80 + pos_x_start) && !rt_x_end) {
                    rise_time_x_end = ros::Time::now().toSec();
                    rt_x_end = true;
                    rise_time_x = rise_time_x_end - rise_time_x_start;
                }
                // Compute max overshoot for x axis
                if (x>=2+pos_x_start) {
                    max_os_x = std::max(max_os_x, x-2-pos_x_start);
                }

                ROS_INFO("Maximum overshoot x = %3.3lf", max_os_x);
                ROS_INFO("Rise Time x = %3.3lf", rise_time_x);

                // Compute the rise time for y axis
                if ((y >= 0.20 + pos_y_start) && !rt_y_start) {
                    rise_time_y_start = ros::Time::now().toSec();
                    rt_y_start = true;
                } else if ((y >= 1.80 + pos_y_start) && !rt_y_end) {
                    rise_time_y_end = ros::Time::now().toSec();
                    rt_y_end = true;
                    rise_time_y = rise_time_y_end - rise_time_y_start;
                }
                // Compute max overshoot for x axis
                if (y>=2+pos_y_start) {
                    max_os_y = std::max(max_os_y, y-2-pos_y_start);
                }

                ROS_INFO("Maximum overshoot y = %3.3lf", max_os_y);
                ROS_INFO("Rise Time y = %3.3lf", rise_time_y);

                // write data to file
                // time vs dist
                data_file_lin << prev_time << "\t" << (x - pos_x_start) << std::endl;
            }   
        }

        // wait for rate
        rate.sleep();
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.linear.y = 0;
    msg_cmd.linear.z = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    // disable motors
    ROS_INFO(" HMOVE : Motors Disabled");
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv);

    ROS_INFO(" HMOVE : ===== END =====");

    return 0;
}
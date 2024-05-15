#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <fstream>
#include "common.hpp"

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;
    std::ofstream data_file;
    data_file.open("/home/khin/team16/data.txt");
    if (data_file.fail()) ROS_INFO("File Open Failed");
    else ROS_INFO("File Open Success");
    // Get ROS Parameters
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    double lin_error = dist_euc(pos_rbt.x, pos_rbt.y, target.x, target.y); 
    double lin_error_sum = 0, lin_error_prev = lin_error;
    double ang_error = limit_angle(atan2(target.y-pos_rbt.y, target.x-pos_rbt.x) - ang_rbt);
    if (abs(ang_error)>M_PI/2) ang_error = (ang_error<0)?(ang_error+M_PI):(ang_error-M_PI);
    double ang_error_sum = 0, ang_error_prev = ang_error;
    double lin_error_derr = 0, ang_error_derr = 0;
    // robot is initially stationary so there is no control signal at t=0
    double lin_control_signal = 0, lin_control_signal_prev = 0;
    double ang_control_signal = 0, ang_control_signal_prev = 0;
    // variables for experimentation
    double rise_time, rise_time_start, rise_time_end;
    double max_lin_overshoot = -10, max_ang_overshoot = -10;
    double init_rbt_x = pos_rbt.x, init_rbt_y = pos_rbt.y;
    // boolean variables to take measurements only once in the loop
    bool take_rise_time_start = true, take_rise_time_end = true, passed_neg_x_axis = false;
    double init_ang_rbt = ang_rbt, max_ang_rbt = ang_rbt;
    // variables to compute distance travelled
    double total_dist_travelled = 0, rbt_x_prev = pos_rbt.x, rbt_y_prev = pos_rbt.y; 

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();
            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////
            if (verbose) { // code for experimentation
                // measure start and end of rise time
                //if (take_rise_time_start && dist_euc(pos_rbt.x, pos_rbt.y, init_rbt_x, init_rbt_y)>=0.02) {
                if (take_rise_time_start && abs(ang_rbt - init_ang_rbt)>= M_PI*0.1) {
                    take_rise_time_start = false;
                    rise_time_start = ros::Time::now().toSec();
                    ROS_INFO("RS: %6.8f", rise_time_start);
                }
                //if (take_rise_time_end && dist_euc(pos_rbt.x, pos_rbt.y, init_rbt_x, init_rbt_y)>=0.18) {
                if (take_rise_time_end && abs(ang_rbt - init_ang_rbt)>= M_PI*0.9) {
                    take_rise_time_end = false;
                    rise_time_end = ros::Time::now().toSec();
                    ROS_INFO("RE: %6.8f", rise_time_end);
                    rise_time = rise_time_end - rise_time_start;
                }

                // checks if robot overshot the negative x axis
                if (ang_rbt<0) {
                    passed_neg_x_axis = true;
                    max_ang_rbt = -M_PI;
                }
                // max_ang_rbt is the largest angle robot made wrt -x axis in bottom right quadrant
                if (ang_rbt > 0 && !passed_neg_x_axis || ang_rbt < 0 && passed_neg_x_axis) max_ang_rbt = std::max(max_ang_rbt, ang_rbt);
                
                if (!take_rise_time_start && !take_rise_time_end) {
                    ROS_INFO("Rise Time: %6.5f", rise_time);
                    // distance
                    max_lin_overshoot = std::max(max_lin_overshoot, dist_euc(pos_rbt.x, pos_rbt.y, init_rbt_x, init_rbt_y)-0.2);
                    //ROS_INFO("Max linear overshoot: %6.3f", max_lin_overshoot);
                    // angular
                    max_ang_overshoot = std::max(max_ang_overshoot, passed_neg_x_axis?(M_PI+max_ang_rbt):(max_ang_rbt - M_PI));
                    ROS_INFO("Max angular overshoot: %6.3f", max_ang_overshoot);
                }
            }

            // PID for linear velocity
            lin_error = dist_euc(pos_rbt.x, pos_rbt.y, target.x, target.y);
            lin_error_sum += lin_error*dt;
            lin_error_derr = (lin_error-lin_error_prev) / dt;
            lin_control_signal = Kp_lin*lin_error + Ki_lin*lin_error_sum + Kd_lin*lin_error_derr;
            lin_error_prev = lin_error;

            // PID for angular velocity
            ang_error = limit_angle(atan2(target.y-pos_rbt.y, target.x-pos_rbt.x) - ang_rbt);
            
            // lab 2.2.1 suggestion: reverse if error is large
            bool reverse_motion = false;
            if (abs(ang_error)>M_PI/2) {
                ang_error = (ang_error<0)?(ang_error+M_PI):(ang_error-M_PI);
                reverse_motion = true;
            }
            ang_error_sum += ang_error*dt;
            ang_error_derr = (ang_error - ang_error_prev) / dt;
            ang_control_signal = Kp_ang*ang_error + Ki_ang*ang_error_sum + Kd_ang*ang_error_derr;
            ang_error_prev = ang_error;
            
            // Coupling Angular Error with Linear Velocity
            // piecewise function used 0.5*(cos4x +1) for |x|<pi/4, 0 other wise
            lin_control_signal *= (abs(ang_error)>M_PI/4?0: (0.5*(cos(4*ang_error)+1)));
            lin_control_signal *= (reverse_motion?-1:1);

            // Accelerations
            double ang_accel = (ang_control_signal - ang_control_signal_prev) / dt;
            double lin_accel = (lin_control_signal - lin_control_signal_prev) / dt;

            //ROS_INFO("before ang_accel: %6.3f", ang_accel);
            //ROS_INFO("before lin_accel: %6.3f", lin_accel);
            
            // Adding Constraints
            ang_accel = sat(ang_accel, max_ang_acc);
            lin_accel = sat(lin_accel, max_lin_acc);
            cmd_ang_vel = sat(cmd_ang_vel+ang_accel*dt, max_ang_vel);
            cmd_lin_vel = sat(cmd_lin_vel+lin_accel*dt, max_lin_vel);
            
            //ROS_INFO("after ang_accel: %6.3f", ang_accel);
            //ROS_INFO("after lin_accel: %6.3f", lin_accel);
            
            ang_control_signal_prev = cmd_ang_vel;
            lin_control_signal_prev = cmd_lin_vel;

            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel;
            pub_cmd.publish(msg_cmd);

            //ROS_INFO("Angular error %6.3f", ang_error);
            //ROS_INFO("Linear error: %6.3f", lin_error);
            //ROS_INFO("cmd_lin_vel: %6.3f", cmd_lin_vel);
            //ROS_INFO("cmd_ang_vel: %6.3f", cmd_ang_vel);

            total_dist_travelled += dist_euc(pos_rbt.x, pos_rbt.y, rbt_x_prev, rbt_y_prev);
            rbt_x_prev = pos_rbt.x; rbt_y_prev = pos_rbt.y;
            ROS_INFO_THROTTLE(2, "Total Distance Travelled: %6.3f", total_dist_travelled);

            // verbose
            if (verbose)
            {
                ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f)", cmd_lin_vel, cmd_ang_vel);
            }
            // write data to file
            /*
            data_file << ros::Time::now().toSec() << "\t" << dist_euc(pos_rbt.x, pos_rbt.y, init_rbt_x, init_rbt_y) << "\t" 
                    << ((ang_rbt>0)?(ang_rbt-2*M_PI):(ang_rbt)) << std::endl;

            */
            
            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO("Total Distance Travelled: %6.3f", total_dist_travelled);

    data_file.close();
    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}

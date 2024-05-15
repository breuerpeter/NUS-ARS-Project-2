#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <errno.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include "common.hpp"
#include "trajectory.hpp"
#include <fstream>

#define NaN std::numeric_limits<double>::quiet_NaN()

enum HectorState
{
    TAKEOFF,
    LAND,
    TURTLE,
    START,
    GOAL,
    TUNING,
    CALIBRATION,
    TRAJECTORY
};
std::string to_string(HectorState state)
{
    switch (state)
    {
    case TAKEOFF:
        return "TAKEOFF";
    case LAND:
        return "LAND";
    case TURTLE:
        return "TURTLE";
    case START:
        return "START";
    case GOAL:
        return "GOAL";
    default:
        return "??";
    }
}

bool verbose, collect_data, tune_pid;
double initial_x, initial_y, initial_z;
double x = NaN, y = NaN, z = NaN, a = NaN;

void cbHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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
double turtle_x = NaN, turtle_y = NaN;
void cbTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    turtle_x = p.x;
    turtle_y = p.y;
}
double vx = NaN, vy = NaN, vz = NaN, va = NaN;
void cbHVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->linear.z;
    va = msg->angular.z;
}

// get the path of the turtle
nav_msgs::Path::ConstPtr turtle_path = nullptr;
bool is_new_turtle_path = false;
void cbTTraj(const nav_msgs::Path::ConstPtr &msg)
{   
    if (turtle_path != msg) is_new_turtle_path = true;
    turtle_path = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_main");
    ros::NodeHandle nh;

    // Make sure motion and move can run (fail safe)
    nh.setParam("run", true); // turns off other nodes

    double main_iter_rate;
    if (!nh.param("main_iter_rate", main_iter_rate, 25.0))
        ROS_WARN(" HMAIN : Param main_iter_rate not found, set to 25");
    if (!nh.param("initial_x", initial_x, 0.0))
        ROS_WARN(" HMAIN : Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", initial_y, 0.0))
        ROS_WARN(" HMAIN : Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", initial_z, 0.178))
        ROS_WARN(" HMAIN : Param initial_z not found, set initial_z to 0.178");
    double height;
    if (!nh.param("height", height, 2.0))
        ROS_WARN(" HMAIN : Param initial_z not found, set to 5");
    double look_ahead;
    if (!nh.param("look_ahead", look_ahead, 1.0))
        ROS_WARN(" HMAIN : Param look_ahead not found, set to 1");
    double close_enough;
    if (!nh.param("close_enough", close_enough, 0.1))
        ROS_WARN(" HMAIN : Param close_enough not found, set to 0.1");
    double average_speed;
    if (!nh.param("average_speed", average_speed, 2.0))
        ROS_WARN(" HMAIN : Param average_speed not found, set to 2.0");
    if (!nh.param("verbose_main", verbose, true))
        ROS_WARN(" HMAIN : Param verbose_main not found, set to false");
    if (!nh.param("collect_data", collect_data, false))
        ROS_WARN("HMAIN: Param collect_data not found, set to false");
    if (!nh.param("tune_pid", tune_pid, false))
        ROS_WARN("HMAIN: Param tune_pid not found, set to false");
    double target_dt;
    if (!nh.param("target_dt", target_dt, 0.025))
            ROS_WARN(" TMAIN : Param target_dt not found, set to 0.025");
    double turtle_dt;
    if (!nh.param("turtle_dt", turtle_dt, 0.04))
            ROS_WARN(" TMAIN : Param turtle_dt not found, set to 0.04");
    bool enable_spline;
    if (!nh.param("enable_spline", enable_spline, true))
            ROS_WARN(" TMAIN : Param enable_spline not found, set to true");
    bool cubic_trajectory;
    if (!nh.param("cubic_trajectory", cubic_trajectory, false))
            ROS_WARN(" TMAIN : Param cubic_trajectory not found, set to false");
    bool sonar_experiments;
    if (!nh.param("sonar_experiments", sonar_experiments, false))
            ROS_WARN(" TMAIN : Param sonar_experiments not found, set to false");
    bool traj_experiments;
    if (!nh.param("traj_experiments", traj_experiments, false))
            ROS_WARN(" TMAIN : Param traj_experiments not found, set to false");

    // --------- Open file for Hector Velocity ----------
    std::ofstream data_file_vel;
    // replace with local file directory
    // file path as follows "/home/<user>/<project folder>/Lin_Vel_data.txt"
    data_file_vel.open("/home/ubuntu2004/EE4308_Project2/Lin_Vel_data.txt");
    if (data_file_vel.fail()) ROS_INFO("H Traj Vel: File Open Failed");
    else ROS_INFO("H Traj Vel: File Open Success");

    // get the final goal position of turtle
    std::string goal_str;
    double goal_x = NaN, goal_y = NaN;
    if (nh.param("/turtle/goals", goal_str, std::to_string(initial_x) + "," + std::to_string(initial_y))) // set to initial hector positions
    {
        char goal_str_tok[goal_str.length() + 1];
        strcpy(goal_str_tok, goal_str.c_str()); // to tokenise --> convert to c string (char*) first
        char *tok = strtok(goal_str_tok, " ,");
        try
        {
            while (tok != nullptr)
            {
                goal_x = strtod(tok, nullptr);
                goal_y = strtod(strtok(nullptr, " ,"), nullptr);
                tok = strtok(nullptr, " ,");
            }
            ROS_INFO(" HMAIN : Last Turtle Goal is (%lf, %lf)", goal_x, goal_y);
        }
        catch (...)
        {
            ROS_ERROR(" HMAIN : Invalid Goals: %s", goal_str.c_str());
            ros::shutdown();
            return 1;
        }
    }
    else
        ROS_WARN(" HMAIN : Param goal not found, set to %s", goal_str.c_str());

    // --------- Subscribers ----------
    ros::Subscriber sub_hpose = nh.subscribe("pose", 1, &cbHPose);
    ros::Subscriber sub_tpose = nh.subscribe("/turtle/pose", 1, &cbTPose);
    ros::Subscriber sub_hvel = nh.subscribe("velocity", 1, &cbHVel);
    ros::Subscriber sub_ttraj = nh.subscribe("/turtle/trajectory", 1, &cbTTraj);

    // --------- Publishers ----------
    ros::Publisher pub_target = nh.advertise<geometry_msgs::PointStamped>("target", 1, true);
    geometry_msgs::PointStamped msg_target;
    msg_target.header.frame_id = "world";
    ros::Publisher pub_rotate = nh.advertise<std_msgs::Bool>("rotate", 1, true);
    std_msgs::Bool msg_rotate;
    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    nav_msgs::Path msg_traj;
    msg_traj.header.frame_id = "world";

    // --------- Wait for Topics ----------
    while (ros::ok() && nh.param("run", true) && (std::isnan(x) || std::isnan(turtle_x) || std::isnan(vx))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce();                                                                                    // update the topics

    // --------- Main loop ----------
    ROS_INFO(" HMAIN : ===== BEGIN =====");

    HectorState state = TAKEOFF;
    if (sonar_experiments || traj_experiments) {
        state =  TRAJECTORY;
    } else if (collect_data && !sonar_experiments) {
        state = CALIBRATION;
    } else if (tune_pid) {
        state = TUNING;
    }
    // x, y coordinates of hector at start of linear experiments
    double expt_x, expt_y;
    std::vector<Position> trajectory = {};
    bool targetHeightReached = false;
    ros::Rate rate(main_iter_rate);
    int target_idx = 0; // index of trajectory for hector
    double turtle_idx = 0; // turtle trajectory index
    Position predicted_turtle_pose;
    Position look_ahead_pos; // next intermediate waypoint for hector trajectory

    // trajectory tuning variables
    int trajectory_goal_idx = 0; // index of goals when performing trajectory tuning
    std::vector<Position> trajectory_goals = {Position(-5,-3)};//{Position(-5,-3), Position(1,-6), Position(-4, -9.5)};
    double dt;
    double prev_time = ros::Time::now().toSec();

    while (ros::ok() && nh.param("run", true))
    {
        // get topics
        ros::spinOnce();

        //// IMPLEMENT ////
        if (state == TAKEOFF)
        {   // Initial State
            // Disable Rotate
            msg_rotate.data = false;
            pub_rotate.publish(msg_rotate);

            // Set target to be directly above
            msg_target.point.z = height;
            msg_target.point.x = initial_x;
            msg_target.point.y = initial_y;
            pub_target.publish(msg_target);
            
            // State transition when the height is reached
            if (z>=height) {
                state = TURTLE;
            }
        }
        else if (state == TURTLE)
        {   // flying to turtlebot
            // Enable Rotate
            msg_rotate.data = true;
            pub_rotate.publish(msg_rotate);

            // new trajectory from turtle
            if (is_new_turtle_path) {
                is_new_turtle_path = false;
                // smaller indexes are nearer the goal, larger indexes are nearer the turtle
                turtle_idx = int(turtle_path->poses.size() - 1);
                if (turtle_idx > look_ahead / turtle_dt) // choose a point that the turtle is going to be at after look_ahead seconds
                    turtle_idx -= (look_ahead / turtle_dt);
            }
            // keep index to be always greater than or equal to 0
            turtle_idx = std::max(turtle_idx, 0.0);
            // get the look ahead pose of turtle
            geometry_msgs::PoseStamped turtle_look_ahead_pose;
            turtle_look_ahead_pose = turtle_path->poses[int(turtle_idx)];
            predicted_turtle_pose = Position(turtle_look_ahead_pose.pose.position.x, turtle_look_ahead_pose.pose.position.y);

            // Set target to be predicted turtle pose
            msg_target.point.z = height;
            msg_target.point.x = predicted_turtle_pose.x;
            msg_target.point.y = predicted_turtle_pose.y;

            //pub_target.publish(msg_target);

            // State transition when close enough to turtle
            if (dist_euc(x, y, turtle_x, turtle_y) <= close_enough) {
                state = GOAL;
            }
        }
        else if (state == START)
        {   // flying to hector's starting position
            // Enable Rotate
            msg_rotate.data = true;
            pub_rotate.publish(msg_rotate);

            // Set target to be starting position
            msg_target.point.z = height;
            msg_target.point.x = initial_x;
            msg_target.point.y = initial_y;
            //pub_target.publish(msg_target);

            // State transition when close enough to starting position
            if (dist_euc(x, y, initial_x, initial_y) <= close_enough) {
                if (!nh.param("/turtle/run", false))
                { // use this if else case to track when the turtle reaches the final goal
                    state = LAND;
                } else {
                    state = TURTLE;
                }
            }
        }
        else if (state == GOAL)
        {   // flying to goal
            // Enable Rotate
            msg_rotate.data = true;
            pub_rotate.publish(msg_rotate);
            
            // Set target to be goal
            msg_target.point.z = height;
            msg_target.point.x = goal_x;
            msg_target.point.y = goal_y;
            pub_target.publish(msg_target);

            // State transition when close enough to goal
            if (dist_euc(x, y, goal_x, goal_y) <= close_enough) {
                state = START;
            }
        }
        else if (state == LAND)
        {   // reached hector's starting position, and trying to land. Can disable rotation.
            // Disable Rotate
            msg_rotate.data = false;
            pub_rotate.publish(msg_rotate);
            // Set target to be starting position
            msg_target.point.z = initial_z;
            msg_target.point.x = initial_x;
            msg_target.point.y = initial_y;
            pub_target.publish(msg_target);
            if (dist_euc(x, y, initial_x, initial_y) <= close_enough && abs(z - initial_z) <= close_enough)
                break;
        }
        else if (state == TUNING) {
            // unreachable state used to tune the PID controllers
            bool tuneVertical = false;
            // tune z axis
            if (tuneVertical) {
                // Disable Rotate
                msg_rotate.data = false;
                pub_rotate.publish(msg_rotate);

                // Set target to be directly above
                msg_target.point.z = height;
                msg_target.point.x = initial_x;
                msg_target.point.y = initial_y;
                pub_target.publish(msg_target);
            } else {
                // tune horizontal movement
                if (!targetHeightReached) {
                    // Disable Rotate
                    msg_rotate.data = false;
                    pub_rotate.publish(msg_rotate);
                } else {
                    // Enable Rotate
                    msg_rotate.data = false;
                    pub_rotate.publish(msg_rotate);
                }
                if (!targetHeightReached && z>=2) {
                    expt_x = x;
                    expt_y = y;
                }
                msg_target.point.z = height;

                if (z>=2) targetHeightReached = true;
                // Set target to be 2 units away
                if (targetHeightReached) { // reached target height
                    msg_target.point.x = expt_x+2;
                    msg_target.point.y = expt_y;
                } else { // yet to reach target height
                    msg_target.point.x = initial_x;
                    msg_target.point.y = initial_y;
                }
                pub_target.publish(msg_target);
            }
        } else if (state == CALIBRATION) {
            // unreachable state used to tune the biases and variances
            // just by not publishing any messages, the Hector wont move
        } else if (state == TRAJECTORY) {
            // unreachable state used to tune the trajectory
            
            dt = ros::Time::now().toSec() - prev_time;
            prev_time += dt;

            // fly to height of 2m 
            if (!targetHeightReached) {
                // Disable Rotate
                msg_rotate.data = false;
            } else {
                // Enable Rotate
                msg_rotate.data = true;
            }
            msg_target.point.z = height;

            if (z>=2) targetHeightReached = true;
            // Set target to be predefined goals
            if (targetHeightReached) { // reached target height
                Position target_point = trajectory_goals[std::min(trajectory_goal_idx, int(trajectory_goals.size()-1))];
                // if near target move to next target
                if (dist_euc(Position(x,y), target_point) <= close_enough
                    && trajectory_goal_idx < trajectory_goals.size()) trajectory_goal_idx++;
                msg_target.point.x = target_point.x;
                msg_target.point.y = target_point.y;
                // if all goals reached, end
                if (trajectory_goal_idx>=trajectory_goals.size()) {
                    msg_target.point.z = initial_z;
                    // Disable Rotate
                    msg_rotate.data = false;
                    if (abs(z - initial_z) <= close_enough) break;
                    pub_rotate.publish(msg_rotate);
                    pub_target.publish(msg_target);
                    rate.sleep();
                    continue;
                }
            } else { // yet to reach target height
                msg_target.point.x = initial_x;
                msg_target.point.y = initial_y;
                //pub_target.publish(msg_target);
                //rate.sleep();
                //continue;
            }
            // log data
            data_file_vel << vx << "\t" << vy << "\t" << sqrt(vx*vx + vy*vy) << std::endl; 
            
            pub_rotate.publish(msg_rotate);
        }
        
        // forward the prediction of turtlebot's position        
        turtle_idx -= (target_dt / turtle_dt);

        // trajectory only required during TURTLE, START and GOAL states 
	    if (state == TURTLE || state == START || state == GOAL || state == TRAJECTORY) {
            
            // different targets, replan
            trajectory = generate_trajectory(Position(x, y), Position(msg_target.point.x, msg_target.point.y), 
                                                        average_speed, target_dt, cubic_trajectory, enable_spline);                                              
            
            // publish trajectroy to trajectory topic
            msg_traj.poses.clear();
            
            for (Position &pos : trajectory) {

                msg_traj.poses.push_back(geometry_msgs::PoseStamped()); // insert a posestamped initialised to all 0
                msg_traj.poses.back().pose.position.x = pos.x;
                msg_traj.poses.back().pose.position.y = pos.y;
                msg_traj.poses.back().pose.position.z = height;
            }
            
            target_idx = trajectory.size()-1; // last entry
            //choose a point further away to avoid intermittent stopping
            target_idx = std::min(target_idx, int(look_ahead / target_dt));
            look_ahead_pos = trajectory[target_idx];

            // update target
            msg_target.point.x = look_ahead_pos.x;              
            msg_target.point.y = look_ahead_pos.y;             
            msg_target.point.z = height; 

            pub_target.publish(msg_target);
            pub_traj.publish(msg_traj);
        } 
	
        if (verbose)
            ROS_INFO_STREAM(" HMAIN : " << to_string(state));

        rate.sleep();
    }

    nh.setParam("run", false); // turns off other nodes
    ROS_INFO(" HMAIN : ===== END =====");
    return 0;
}

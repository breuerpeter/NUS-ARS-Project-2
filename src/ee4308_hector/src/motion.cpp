#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> // publish to pose topic
#include <geometry_msgs/Vector3Stamped.h>            // subscribe to magnetic topic
#include <sensor_msgs/Imu.h>                         // subscribe to imu topic
#include <sensor_msgs/NavSatFix.h>                   // subscribe to GPS
#include <hector_uav_msgs/Altimeter.h>               // subscribe to barometer
#include <sensor_msgs/Range.h>                       // subscribe to sonar
#include <nav_msgs/Odometry.h>                       // subscribe to ground truth topic
#include <std_srvs/Empty.h>                          // Service to calrbrate motors
#include <opencv2/core/core.hpp>
#include "common.hpp"
#include <fstream>

#define NaN std::numeric_limits<double>::quiet_NaN()

// global parameters to be read from ROS PARAMs
bool verbose, use_ground_truth, enable_baro, enable_magnet, enable_sonar, enable_gps, collect_data, plot_data, enable_sonar_correction;

// others
bool ready = false; // signal to topics to begin

// logging variables
std::ofstream data_file_gps, data_file_baro, data_file_magnet, data_file_sonar, data_file_ekf;

// ----------------------------------------
// PREDICTION WITH IMU
// ----------------------------------------
const double G = 9.8;
double prev_imu_t = 0;
cv::Matx21d X = {0, 0},
            Y = {0, 0},
            Z = {0, 0},
            A = {0, 0}; // cv::Matx<double, 2, 1>
cv::Matx22d P_x = cv::Matx22d::ones(),
            P_y = cv::Matx22d::ones(),
            P_z = cv::Matx22d::ones(),
            P_a = cv::Matx22d::ones();

double ua = NaN,
       ux = NaN,
       uy = NaN,
       uz = NaN;
double qa, qx, qy, qz;

void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!ready)
    {
        prev_imu_t = msg->header.stamp.toSec();
        return;
    }

    // calculate time
    double imu_t = msg->header.stamp.toSec();
    double imu_dt = imu_t - prev_imu_t;
    prev_imu_t = imu_t;

    // read inputs
    ua = msg->angular_velocity.z;
    ux = msg->linear_acceleration.x;
    uy = msg->linear_acceleration.y;
    uz = msg->linear_acceleration.z;

    // Jacobian matrices
    cv::Matx22d Fx = {1, imu_dt,
                      0, 1};
    cv::Matx22d Wx = {-0.5 * pow(imu_dt, 2) * cos(A(0)), 0.5 * pow(imu_dt, 2) * sin(A(0)),
                      -imu_dt * cos(A(0)), imu_dt * sin(A(0))};
    cv::Matx21d Uxy = {ux, uy};

    cv::Matx22d Fy = {1, imu_dt,
                      0, 1};
    cv::Matx22d Wy = {-0.5 * pow(imu_dt, 2) * sin(A(0)), -0.5 * pow(imu_dt, 2) * cos(A(0)),
                      -imu_dt * sin(A(0)), -imu_dt * cos(A(0))};

    cv::Matx22d Fz = {1, imu_dt,
                      0, 1};
    cv::Matx21d Wz = {0.5 * pow(imu_dt, 2),
                      imu_dt};
    double Uz = uz - G;

    cv::Matx22d Fa = {1, 0,
                      0, 0};
    cv::Matx21d Wa = {imu_dt,
                      1};

    // covariance matrices
    cv::Matx22d Qxy = {qx, 0,
                       0, qy};

    // prior update
    X = Fx * X + Wx * Uxy;
    Y = Fy * Y + Wy * Uxy;
    Z = Fz * Z + Wz * Uz;
    A = Fa * A + Wa * ua;

    P_x = Fx * P_x * Fx.t() + Wx * Qxy * Wx.t();
    P_y = Fy * P_y * Fy.t() + Wy * Qxy * Wy.t();
    P_z = Fz * P_z * Fz.t() + Wz * qz * Wz.t();
    P_a = Fa * P_a * Fa.t() + Wa * qa * Wa.t();
}

// ----------------------------------------
// GPS CORRECTION
// ----------------------------------------
// https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
cv::Matx31d GPS = {NaN, NaN, NaN};
cv::Matx31d initial_pos = {NaN, NaN, NaN}; // written below in main. no further action needed.
cv::Matx31d initial_ECEF = {NaN, NaN, NaN};
const double DEG2RAD = M_PI / 180;
const double RAD_POLAR = 6356752.3;
const double RAD_EQUATOR = 6378137;
const double e2 = 1 - pow(RAD_POLAR, 2) / pow(RAD_EQUATOR, 2);
double r_gps_x, r_gps_y, r_gps_z;
cv::Matx12d H = {1, 0};
cv::Matx<double, 1, 1> V = {1};
int gps_cnt = 0;

void cbGps(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (!ready)
        return;

    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    // convert to radians
    lat *= DEG2RAD;
    lon *= DEG2RAD;

    // coordinates in ECEF frame
    double N = RAD_EQUATOR / sqrt(1 - e2 * pow(sin(lat), 2));
    cv::Matx31d ECEF = {(N + alt) * cos(lat) * cos(lon),
                        (N + alt) * cos(lat) * sin(lon),
                        ((1 - e2) * N + alt) * sin(lat)};

    // run once at beginning
    if (std::isnan(initial_ECEF(0)))
    {
        initial_ECEF = ECEF;
        return;
    }

    // coordinates in NED frame
    cv::Matx33d Ren = {-sin(lat) * cos(lon), -sin(lon), -cos(lat) * cos(lon),
                       -sin(lat) * sin(lon), cos(lon), -cos(lat) * sin(lon),
                       cos(lat), 0, -sin(lat)};
    cv::Matx31d NED = Ren.t() * (ECEF - initial_ECEF);

    // coordinates in world (gazebo) frame
    cv::Matx33d Rmn = {1, 0, 0,
                       0, -1, 0,
                       0, 0, -1};
    GPS = Rmn * NED + initial_pos;

    // measurement updates
    cv::Matx21d Kx = P_x * H.t() * (H * P_x * H.t() + V * r_gps_x * V.t()).inv();
    X = X + Kx * (GPS(0) - X(0));
    P_x = P_x - Kx * H * P_x;

    cv::Matx21d Ky = P_y * H.t() * (H * P_y * H.t() + V * r_gps_y * V.t()).inv();
    Y = Y + Ky * (GPS(1) - Y(0));
    P_y = P_y - Ky * H * P_y;

    cv::Matx21d Kz = P_z * H.t() * (H * P_z * H.t() + V * r_gps_z * V.t()).inv();
    Z = Z + Kz * (GPS(2) - Z(0));
    P_z = P_z - Kz * H * P_z;

    if (collect_data && gps_cnt < 200)
    {
        data_file_gps << GPS(0) << "," << GPS(1) << "," << GPS(2) << std::endl;
        gps_cnt++;
    }
    if (gps_cnt >= 200)
    {
        ROS_INFO("GPS data collection done");
    }
}

// ----------------------------------------
// MAGNETOMETER CORRECTION
// ----------------------------------------
double a_mgn = NaN;
double r_mgn_a;
int magnet_cnt = 0;

void cbMagnet(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    if (!ready)
        return;

    double mx = msg->vector.x;
    double my = msg->vector.y;

    // measurement update
    a_mgn = -atan2(my, mx);

    cv::Matx21d Ka = P_a * H.t() * (H * P_a * H.t() + V * r_mgn_a * V.t()).inv();
    A = A + Ka * limit_angle(a_mgn - A(0));
    P_a = P_a - Ka * H * P_a;

    // limit to [-pi, pi]
    A(0) = limit_angle(A(0));

    if (collect_data && magnet_cnt < 200)
    {
        data_file_magnet << a_mgn << std::endl;
        magnet_cnt++;
    }
    if (magnet_cnt >= 200)
    {
        ROS_INFO("Magnet data collection done");
    }
}

// ----------------------------------------
// BAROMETER CORRECTION
// ----------------------------------------
double z_bar = NaN;
double r_bar_z;
double b_bar_init;
double p_bar_z_init;
cv::Matx31d Z_bar = {0, 0, NaN}; // augmented state vector
cv::Matx33d P_z_bar = {0, 0, 0,
                       0, 0, 0,
                       0, 0, NaN}; // augmented covariance matrix

int baro_cnt = 0;

void cbBaro(const hector_uav_msgs::Altimeter::ConstPtr &msg)
{
    if (!ready)
        return;

    z_bar = msg->altitude;

    // update augmented matrices (for measurement update)
    Z_bar(0) = Z(0);
    Z_bar(1) = Z(1);

    if (std::isnan(Z_bar(2)))
    {
        Z_bar(2) = b_bar_init; // initialization
    }

    P_z_bar(0, 0) = P_z(0, 0);
    P_z_bar(0, 1) = P_z(0, 1);
    P_z_bar(1, 0) = P_z(1, 0);
    P_z_bar(1, 1) = P_z(1, 1);

    if (std::isnan(P_z_bar(2, 2)))
    {
        P_z_bar(2, 2) = p_bar_z_init; // initialization
    }

    /* ROS_INFO_STREAM("Z_bar:" << Z_bar << std::endl);
    ROS_INFO_STREAM("P_z:" << P_z << std::endl);
    ROS_INFO_STREAM("P_z_bar:" << P_z_bar << std::endl); */

    // measurement update
    cv::Matx13d Hbar = {1, 0, 1};

    cv::Matx31d Kz_bar = P_z_bar * Hbar.t() * (Hbar * P_z_bar * Hbar.t() + V * r_bar_z * V.t()).inv();
    Z_bar = Z_bar + Kz_bar * (z_bar - (Z_bar(0) + Z_bar(2)));
    P_z_bar = P_z_bar - Kz_bar * Hbar * P_z_bar;

    /* ROS_INFO_STREAM("Kz_bar:" << Kz_bar << std::endl);
    ROS_INFO_STREAM("Z_bar:" << Kz_bar << std::endl);
    ROS_INFO_STREAM("P_z_bar:" << P_z_bar << std::endl); */

    // update unaugmented matrices (for prior update)
    Z(0) = Z_bar(0);
    Z(1) = Z_bar(1);

    P_z(0, 0) = P_z_bar(0, 0);
    P_z(0, 1) = P_z_bar(0, 1);
    P_z(1, 0) = P_z_bar(1, 0);
    P_z(1, 1) = P_z_bar(1, 1);

    if (collect_data && baro_cnt < 200)
    {
        data_file_baro << z_bar << std::endl;
        baro_cnt++;
    }
    if (baro_cnt >= 200)
    {
        ROS_INFO("Baro data collection done");
    }
}

// ----------------------------------------
// SONAR CORRECTION
// ----------------------------------------
double z_snr = NaN;
double r_snr_z;
int sonar_cnt = 0;
double dt, prev_time, dt_prev;
// check whether Hector is moving at around z = 2m
bool isCruising = false;
int cruising_counter = 0, landing_counter = 0;
double z_snr_prev = NaN, dz_prev = 0;
void cbSonar(const sensor_msgs::Range::ConstPtr &msg)
{
    if (!ready)
        return;

    if (std::isnan(z_snr))
    {
        prev_time = ros::Time::now().toSec();
    }

    z_snr = msg->range;
    if (enable_sonar_correction)
    {
        bool useTrueSnrVal = true;

        dt = ros::Time::now().toSec() - prev_time;
        prev_time += dt;

        // increment counter if height is above, decrease if below
        if (!isCruising && z_snr >= 1.9)
            cruising_counter++;
        else
            cruising_counter = 0;
        // Hector is cruising if there is 5 consecutive counts of height above 1.9m
        if (!isCruising && cruising_counter >= 5)
        {
            isCruising = true;
        }

        // current gradient
        double dz = (z_snr - z_snr_prev) / dt;

        // compute variance of gradients
        double grad_var_prev = 2 * r_snr_z / std::pow(dt_prev, 2);
        double grad_var = 2 * r_snr_z / std::pow(dt, 2);
        // standard deviation of difference of gradients
        double grad_std = sqrt(grad_var + grad_var_prev);

        // check if Hector is currently cruising and
        // if the gradient is negative and changes in gradient is close to each other
        if (isCruising && dz < 0 && abs(dz - dz_prev) <= 3 * grad_std)
            landing_counter++;
        else
            landing_counter = 0;
        // Hector is landing if there are 3 consecutive counts of negative constant gradient
        if (landing_counter >= 3)
            isCruising = false;

        // update gradient
        dz_prev = dz;

        // Hector is cruising and
        // height falls by more than 3 std deviation away in a timestep
        if (isCruising &&
            (abs(z_snr - z_snr_prev) >= 3 * sqrt(2 * r_snr_z) || abs(z_snr - Z(0)) >= 3 * sqrt(2 * r_snr_z)))
        {
            // do not update EKF
            useTrueSnrVal = false;
        }

        // update previous reading
        z_snr_prev = z_snr;

        // update prev dt
        dt_prev = dt;

        if (collect_data && sonar_cnt < 200)
        {
            if (useTrueSnrVal) // time , esitmated pose, z_snr, filtered z_snr
                data_file_sonar << prev_time << "\t" << Z(0) << "\t" << z_snr << "\t" << z_snr << std::endl;
            else
                data_file_sonar << prev_time << "\t" << Z(0) << "\t" << z_snr << "\t" << 2 << std::endl;
            sonar_cnt++;
        }
        if (sonar_cnt >= 200)
        {
            ROS_INFO("Sonar data collection done");
        }

        if (!useTrueSnrVal)
            z_snr = 2;
    }
    // measurement update
    cv::Matx21d Kz_snr = P_z * H.t() * (H * P_z * H.t() + V * r_snr_z * V.t()).inv();
    Z = Z + Kz_snr * (z_snr - Z(0));
    P_z = P_z - Kz_snr * H * P_z;
}

// ----------------------------------------
// GROUND TRUTH
// ----------------------------------------
nav_msgs::Odometry msg_true;
int plot_data_cnt = 0;

void cbTrue(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_true = *msg;
}

// ----------------------------------------
// MEASUREMENT UPDATE WITH GROUND TRUTH
// ----------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_motion");
    ros::NodeHandle nh;

    // --------- parameters ----------
    double motion_iter_rate;
    if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
        ROS_WARN("HMOTION: Param motion_iter_rate not found, set to 50.0");
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN("HMOTION: Param verbose_motion not found, set to false");
    if (!nh.param("initial_x", X(0), 0.0))
        ROS_WARN("HMOTION: Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", Y(0), 0.0))
        ROS_WARN("HMOTION: Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", Z(0), 0.178))
        ROS_WARN("HMOTION: Param initial_z not found, set initial_z to 0.178");
    initial_pos = {X(0), Y(0), Z(0)};
    if (!nh.param("use_ground_truth", use_ground_truth, true))
        ROS_WARN("HMOTION: Param use_ground_truth not found, set use_ground_truth to true");
    if (!nh.param("r_gps_x", r_gps_x, 1.0))
        ROS_WARN("HMOTION: Param r_gps_x not found, set to 1.0");
    if (!nh.param("r_gps_y", r_gps_y, 1.0))
        ROS_WARN("HMOTION: Param r_gps_y not found, set to 1.0");
    if (!nh.param("r_gps_z", r_gps_z, 1.0))
        ROS_WARN("HMOTION: Param r_gps_z not found, set to 1.0");
    if (!nh.param("r_mgn_a", r_mgn_a, 1.0))
        ROS_WARN("HMOTION: Param r_mgn_a not found, set to 1.0");
    if (!nh.param("r_bar_z", r_bar_z, 1.0))
        ROS_WARN("HMOTION: Param r_bar_z not found, set to 1.0");
    if (!nh.param("b_bar_init", b_bar_init, 1000.0))
        ROS_WARN("HMOTION: Param b_bar_init not found, set to 1000.0");
    if (!nh.param("p_bar_z_init", p_bar_z_init, 1000.0))
        ROS_WARN("HMOTION: Param p_bar_z_init not found, set to 1000.0");
    if (!nh.param("r_snr_z", r_snr_z, 1.0))
        ROS_WARN("HMOTION: Param r_snr_z not found, set to 1.0");
    if (!nh.param("qa", qa, 1.0))
        ROS_WARN("HMOTION: Param qa not found, set to 1.0");
    if (!nh.param("qx", qx, 1.0))
        ROS_WARN("HMOTION: Param qx not found, set to 1.0");
    if (!nh.param("qy", qy, 1.0))
        ROS_WARN("HMOTION: Param qy not found, set to 1.0");
    if (!nh.param("qz", qz, 1.0))
        ROS_WARN("HMOTION: Param qz not found, set to 1.0");
    if (!nh.param("enable_baro", enable_baro, true))
        ROS_WARN("HMOTION: Param enable_baro not found, set to true");
    if (!nh.param("enable_magnet", enable_magnet, true))
        ROS_WARN("HMOTION: Param enable_magnet not found, set to true");
    if (!nh.param("enable_sonar", enable_sonar, true))
        ROS_WARN("HMOTION: Param enable_sonar not found, set to true");
    if (!nh.param("enable_gps", enable_gps, true))
        ROS_WARN("HMOTION: Param enable_gps not found, set to true");
    if (!nh.param("collect_data", collect_data, false))
        ROS_WARN("HMOTION: Param collect_data not found, set to false");
    if (!nh.param("plot_data", plot_data, false))
        ROS_WARN("HMOTION: Param plot_data not found, set to false");
    if (!nh.param("enable_sonar_correction", enable_sonar_correction, false))
        ROS_WARN("HMOTION: Param enable_sonar_correction not found, set to false");

    // --------- subscribers ----------
    ros::Subscriber sub_true = nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &cbTrue);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("raw_imu", 1, &cbImu);
    ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("fix", 1, &cbGps);
    if (!enable_gps)
        sub_gps.shutdown();
    ros::Subscriber sub_magnet = nh.subscribe<geometry_msgs::Vector3Stamped>("magnetic", 1, &cbMagnet);
    if (!enable_magnet)
        sub_magnet.shutdown();
    ros::Subscriber sub_baro = nh.subscribe<hector_uav_msgs::Altimeter>("altimeter", 1, &cbBaro);
    if (!enable_baro)
        sub_baro.shutdown();
    ros::Subscriber sub_sonar = nh.subscribe<sensor_msgs::Range>("sonar_height", 1, &cbSonar);
    if (!enable_sonar)
        sub_sonar.shutdown();

    // --------- publishers ----------
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1, true);
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.frame_id = "world";                                               // for rviz
    msg_pose.pose.pose.orientation.x = 0;                                             // no roll
    msg_pose.pose.pose.orientation.y = 0;                                             // no pitch
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("velocity", 1, true); // publish velocity
    geometry_msgs::Twist msg_vel;

    // --------- wait for topics ---------
    ROS_INFO("HMOTION: Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ((std::isnan(ux) && msg_true.header.seq == 0))) // wait for imu and truth only
        ros::spinOnce();                                                                         // update subscribers

    if (!ros::ok())
    { // ROS shutdown
        ROS_INFO("HMOTION: ===== END =====");
        return 0;
    }

    // --------- calibrate Gyro service ----------
    ROS_INFO("HMOTION: Calibrating Gyro...");
    ros::ServiceClient calibrate_gyro = nh.serviceClient<std_srvs::Empty>("raw_imu/calibrate");
    std_srvs::Empty calibrate_gyro_srv;
    if (calibrate_gyro.call(calibrate_gyro_srv))
        ROS_INFO("HMOTION: Calibrated Gyro");
    else
        ROS_WARN("HMOTION: Gyro cannot be calibrated!");

    // --------- data for plotting ---------
    if (plot_data)
    {
        // replace with local file directory
        // file path as follows "/home/<user>/<project folder>/GPS_data.txt"
        data_file_ekf.open("/home/pbreuer/ROS1/ee4308/Project-2/EKF_data.txt");
        if (data_file_ekf.fail())
            ROS_INFO("H EKF: File Open Failed");
    }

    // --------- log bias & variances ----------
    if (collect_data)
    {
        // replace with local file directory
        // file path as follows "/home/<user>/<project folder>/GPS_data.txt"
        data_file_gps.open("/home/pbreuer/ROS1/ee4308/Project-2/GPS_data.txt");
        if (data_file_gps.fail())
            ROS_INFO("H GPS: File Open Failed");
        else
            ROS_INFO("H GPS: File Open Success");
        // file path as follows "/home/<user>/<project folder>/Baro_data.txt"
        data_file_baro.open("/home/pbreuer/ROS1/ee4308/Project-2/Baro_data.txt");
        if (data_file_baro.fail())
            ROS_INFO("H Baro: File Open Failed");
        else
            ROS_INFO("H Baro: File Open Success");
        // file path as follows "/home/<user>/<project folder>/Magnet_data.txt"
        data_file_magnet.open("/home/pbreuer/ROS1/ee4308/Project-2/Magnet_data.txt");
        if (data_file_magnet.fail())
            ROS_INFO("H Magnet: File Open Failed");
        else
            ROS_INFO("H Magnet: File Open Success");
        // file path as follows "/home/<user>/<project folder>/Sonar_data.txt"
        // data_file_sonar.open("/home/pbreuer/ROS1/ee4308/Project-2/Sonar_data.txt");
        data_file_sonar.open("/home/ubuntu2004/EE4308_Project2/Sonar_data.txt");
        if (data_file_sonar.fail())
            ROS_INFO("H Sonar: File Open Failed");
        else
            ROS_INFO("H Sonar: File Open Success");
    }

    // --------- main loop ----------

    ros::Rate rate(motion_iter_rate);
    ROS_INFO("HMOTION: ===== BEGIN =====");
    ready = true;
    while (ros::ok() && nh.param("run", true))
    {
        ros::spinOnce(); // update topics

        auto &tp = msg_true.pose.pose.position;
        auto &q = msg_true.pose.pose.orientation;
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

        // verbose
        if (verbose)
        {
            ROS_INFO("[HM] ---------X-------Y-------Z-------A------");
            ROS_INFO("[HM]  TRUE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", tp.x, tp.y, tp.z, atan2(siny_cosp, cosy_cosp));
            ROS_INFO("[HM] STATE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", X(0), Y(0), Z(0), A(0));
            ROS_INFO("[HM]   GPS(%7.3lf,%7.3lf,%7.3lf, ---- )", GPS(0), GPS(1), GPS(2));
            ROS_INFO("[HM] MAGNT( ----- , ----- , ----- ,%6.3lf)", a_mgn);
            ROS_INFO("[HM]  BARO( ----- , ----- ,%7.3lf, ---- )", z_bar);
            ROS_INFO("[HM] BAROB( ----- , ----- ,%7.3lf, ---- )", Z_bar(2));
            ROS_INFO("[HM] SONAR( ----- , ----- ,%7.3lf, ---- )", z_snr);
        }

        //  publish pose and vel
        if (use_ground_truth)
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position = msg_true.pose.pose.position;
            msg_pose.pose.pose.orientation = msg_true.pose.pose.orientation;
            msg_vel = msg_true.twist.twist;
        }
        else
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position.x = X(0);
            msg_pose.pose.pose.position.y = Y(0);
            msg_pose.pose.pose.position.z = Z(0);
            msg_pose.pose.covariance[0] = P_x(0, 0);  // x cov
            msg_pose.pose.covariance[7] = P_y(0, 0);  // y cov
            msg_pose.pose.covariance[14] = P_z(0, 0); // z cov
            msg_pose.pose.covariance[35] = P_a(0, 0); // a cov
            msg_pose.pose.pose.orientation.w = cos(A(0) / 2);
            msg_pose.pose.pose.orientation.z = sin(A(0) / 2);
            msg_vel.linear.x = X(1);
            msg_vel.linear.y = Y(1);
            msg_vel.linear.z = Z(1);
            msg_vel.angular.z = A(1);
        }
        pub_pose.publish(msg_pose);
        pub_vel.publish(msg_vel);

        if (plot_data && plot_data_cnt < 300)
        {
            data_file_ekf << tp.x << "," << X(0) << "," << tp.y << "," << Y(0) << "," << tp.z << "," << Z(0) << "," << atan2(siny_cosp, cosy_cosp) << "," << A(0) << std::endl;
            plot_data_cnt++;
        }
        if (plot_data && plot_data_cnt >= 300)
        {
            ROS_INFO("Data collection for plotting done");
        }

        rate.sleep();
    }

    // close files
    if (collect_data)
    {
        data_file_gps.close();
        data_file_baro.close();
        data_file_magnet.close();
        data_file_sonar.close();
    }

    if (plot_data)
    {
        data_file_ekf.close();
    }
    ROS_INFO("HMOTION: ===== END =====");
    return 0;
}
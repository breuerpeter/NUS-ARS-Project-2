#include "trajectory.hpp"
#include "common.hpp"
#include <cmath>

std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, double average_speed, double target_dt, bool cubic_trajectory, bool enable_spline) {
    // (1) estimate total duration
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double duration = sqrt(Dx*Dx + Dy*Dy) / average_speed;

    if (!enable_spline) {
        std::vector<Position> trajectory = {pos_begin};
        for (double time = target_dt; time <= duration; time += target_dt) {

            trajectory.emplace_back(
                pos_begin.x + time * Dx / duration, 
                pos_begin.y + time * Dy / duration
            );
        }
        return trajectory;
    }


    // (2) generate cubic / quintic trajectory
    double a0, a1, a2, a3, a4, a5;
    double b0, b1, b2, b3, b4, b5;

    double x_traj,y_traj;
    double curve_speed = average_speed * 0.2;   // lower the speed at the starting and ending points, which is 20%

    if (cubic_trajectory) {
        // Cubic hermite
        a0 = pos_begin.x;
        a1 = curve_speed;
        a2 = (-3.0/pow(duration, 2)) * pos_begin.x + (-2.0/duration) * curve_speed + (3.0/pow(duration, 2)) * pos_end.x + (-1.0/duration) * curve_speed;
        a3 = (2.0/pow(duration, 3)) * pos_begin.x + (1.0/pow(duration, 2)) * curve_speed + (-2.0/pow(duration, 3)) * pos_end.x + (1.0/pow(duration, 2)) * curve_speed;

        b0 = pos_begin.y;
        b1 = curve_speed;
        b2 = (-3.0/pow(duration, 2)) * pos_begin.y + (-2.0/duration) * curve_speed + (3.0/pow(duration, 2)) * pos_end.y + (-1.0/duration) * curve_speed;
        b3 = (2.0/pow(duration, 3)) * pos_begin.y + (1.0/pow(duration, 2)) * curve_speed + (-2.0/pow(duration, 3)) * pos_end.y + (1.0/pow(duration, 2)) * curve_speed;
        
        std::vector<Position> trajectory = {pos_begin};
        for (double time = target_dt; time <= duration; time += target_dt) {

            // cubic hermite
            x_traj = a0 + a1 * time + a2 * pow(time, 2) + a3 * pow(time, 3);
            y_traj = b0 + b1 * time + b2 * pow(time, 2) + b3 * pow(time, 3);

            trajectory.emplace_back(
                x_traj, y_traj
            );
        }
        return trajectory; 
    } else {
        //Qunintic hermite
        a0 = pos_begin.x;
        a1 = curve_speed;
        a2 = 0;  //set acceleration = 0
        a3 = (-10.0/pow(duration, 3)) * pos_begin.x + (-6.0/pow(duration, 2) * curve_speed) + 10.0/pow(duration, 3) * pos_end.x + (-4.0/pow(duration, 2) * curve_speed);
        a4 = (15.0/pow(duration, 4)) * pos_begin.x + (8.0/pow(duration, 3) * curve_speed) + (-15.0/pow(duration, 4) * pos_end.x) + (7.0/pow(duration, 3) * curve_speed);
        a5 = (-6.0/pow(duration, 5)) * pos_begin.x + (-3.0/pow(duration, 4) * curve_speed) + (6.0/pow(duration, 5) * pos_end.x) + (-3.0/pow(duration, 4) * curve_speed);

        b0 = pos_begin.y;
        b1 = curve_speed;
        b2 = 0;
        b3 = (-10.0/pow(duration, 3)) * pos_begin.y + (-6.0/pow(duration, 2) * curve_speed) + 10.0/pow(duration, 3) * pos_end.y + (-4.0/pow(duration, 2) * curve_speed);
        b4 = (15.0/pow(duration, 4)) * pos_begin.y + (8.0/pow(duration, 3) * curve_speed) + (-15.0/pow(duration, 4) * pos_end.y) + (7.0/pow(duration, 3) * curve_speed);
        b5 = (-6.0/pow(duration, 5)) * pos_begin.y + (-3.0/pow(duration, 4) * curve_speed) + (6.0/pow(duration, 5) * pos_end.y) + (-3.0/pow(duration, 4) * curve_speed);
    
        // generate targets for each target_dt
        std::vector<Position> trajectory = {pos_begin};
        for (double time = target_dt; time <= duration; time += target_dt) {          // only generate trajectory up to look_ahead time away

            //quintic hermite
            x_traj = a0 + a1 * time + a2 * pow(time, 2) + a3 * pow(time, 3) + a4 * pow(time, 4) + a5 * pow(time, 5);
            y_traj = b0 + b1 * time + b2 * pow(time, 2) + b3 * pow(time, 3) + b4 * pow(time, 4) + b5 * pow(time, 5);

            trajectory.emplace_back(
                x_traj, y_traj
            );
        }
        return trajectory; 
    }
}

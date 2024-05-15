#include "common.hpp"
#include <vector>

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, double average_speed, double target_dt, bool cubic_trajectory, bool enable_spline);
#endif

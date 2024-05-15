#include "common.hpp"
#include "grid.hpp"

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
std::vector<Position> post_process(std::vector<Position> path, Grid &grid); // returns the turning points
bool lineOfSight(Position curr, Position next, Grid &grid);
std::vector<Position> generate_trajectory_no_spline(Position pos_begin, Position pos_end, double average_speed, double target_dt, Grid & grid);
bool is_safe_trajectory(std::vector<Position> trajectory, Grid & grid);
std::vector<Position> generate_trajectory_spline(Position pos_begin, Position pos_end, double average_speed, double target_dt, Grid & grid, bool use_cubic);
#endif

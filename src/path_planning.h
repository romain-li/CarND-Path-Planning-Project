#ifndef PATH_PLANNING_PATH_PLANNING_H
#define PATH_PLANNING_PATH_PLANNING_H

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "utils.h"

struct Vehicle {
  double x;
  double y;
  double speed;
  double yaw;
  double s;
  double end_s;
  double d;
  int lane;
};

struct Goal {
  double speed;
  double d;
};

class PathPlanner {
  const std::vector<double> map_s;
  const std::vector<double> map_x;
  const std::vector<double> map_y;

  int num_lanes;
  double lane_width;
  double speed_limit = 22.2;
  double acc_limit = 9.9;
  double max_braking_acc = 30;
  double time_delta = 0.02;
  int window_size = 10;
  double safe_distance = 2;
  double score_threshold = 1.1;
  int goal_lane = -1;
  Vehicle current{};
  std::vector<Vehicle> predictions;

public:
  PathPlanner(std::vector<double> &maps_s, std::vector<double> &maps_x, std::vector<double> &maps_y)
      : num_lanes(3), lane_width(4), map_s(maps_s), map_x(maps_x), map_y(maps_y) {}

  ~PathPlanner() = default;

  void update(double x, double y, double yaw, double s, double end_s, double d,
              std::vector<std::vector<double>> &sensor_fusion, double t);

  /**
   * Check if we need do some emergency braking.
   * @return True if something is too close.
   */
  bool isDangerous();

  vector<vector<double>> getPath(vector<double> &previous_path_x, vector<double> &previous_path_y);

private:

  /**
   * Predict the other cars status after T seconds.
   * In this project, all the cars drives straight, so this function only calculate the car's lane and speed.
   */
  void prediction(std::vector<std::vector<double>> &sensor_fusion, double t);

  int getLane(double d);

  Goal getGoal();

  double getSafeDistance(double speed);
};


#endif //PATH_PLANNING_PATH_PLANNING_H

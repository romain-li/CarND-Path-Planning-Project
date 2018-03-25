#include "path_planning.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;

void PathPlanner::update(double x, double y, double yaw, double s, double end_s, double d,
                         std::vector<std::vector<double>> &sensor_fusion, double t) {
  // std::cout << "Update: " << x << ", " << y << ", " << yaw << ", " << end_s << ", " << d << std::endl;
  current.x = x;
  current.y = y;
  current.s = s;
  current.end_s = end_s;
  current.d = d;
  current.yaw = yaw;
  current.lane = getLane(d);
  prediction(sensor_fusion, t);
}

bool PathPlanner::isDangerous() {
  for (auto vehicle:predictions) {
    if ((vehicle.lane == current.lane || abs(vehicle.d - current.d) <= safe_distance) &&
        (vehicle.s <= current.s + safe_distance ||
         vehicle.end_s <= current.end_s + getSafeDistance(vehicle.speed)) &&
        vehicle.s > current.s) {
      return true;
    }
  }
  return false;
}


vector<vector<double>> PathPlanner::getPath(vector<double> &previous_path_x, vector<double> &previous_path_y) {
  Goal goal = getGoal();

  // Code of walk through video
  const unsigned long prev_size = previous_path_x.size();

  if (this->isDangerous()) {
    // Emergency braking !!!
    current.speed -= max_braking_acc * time_delta;
  } else if (current.speed > goal.speed) {
    current.speed -= acc_limit * time_delta;
  } else if (current.speed < goal.speed) {
    current.speed += acc_limit * time_delta;
  }
  if (current.speed < 0) {
    current.speed = 0;
  } else if (current.speed > speed_limit) {
    current.speed = speed_limit;
  }
  // std::cout << "Goal: " << goal.speed << ", " << goal.d << std::endl;

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = current.x;
  double ref_y = current.y;
  const double ref_yaw = deg2rad(current.yaw);

  if (prev_size < 2) {
    const double prev_car_x = current.x - cos(ref_yaw);
    const double prev_car_y = current.y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(current.x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(current.y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    const double ref_x_prev = previous_path_x[prev_size - 2];
    const double ref_y_prev = previous_path_y[prev_size - 2];

    if (ref_x_prev == ref_x) {
      ptsx.push_back(ref_x_prev - 0.00001);  // Avoid spline regression error
    } else {
      ptsx.push_back(ref_x_prev);
    }
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // A simple scalable window, make less prediction and more accuracy
  if (prev_size > 20) {
    window_size -= 1;
  } else {
    window_size += 1;
  }

  const double plan_distance = 30;

  vector<double> next_wp0 = getXY(current.end_s + plan_distance, goal.d, map_s, map_x, map_y);
  vector<double> next_wp1 = getXY(current.end_s + plan_distance * 2, goal.d, map_s, map_x, map_y);
  vector<double> next_wp2 = getXY(current.end_s + plan_distance * 3, goal.d, map_s, map_x, map_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    const double shift_x = ptsx[i] - ref_x;
    const double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  if (current.speed > 0) {
    const double target_x = current.speed * 2;
    const double target_y = s(target_x);
    const double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
    double x_add_on = 0;
    const double N = (target_dist / (time_delta * current.speed));

    for (int i = 1; i <= window_size - previous_path_x.size(); i++) {
      double x_point = x_add_on + (target_x) / N;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
      y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
      x_point += ref_x;
      y_point += ref_y;
      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }
  }

  return {next_x_vals, next_y_vals};
}


void PathPlanner::prediction(std::vector<std::vector<double>> &sensor_fusion, double t) {
  predictions.clear();
  for (const std::vector<double> &vehicle_sensor:sensor_fusion) {
    Vehicle vehicle = Vehicle();
    vehicle.speed = sqrt(pow(vehicle_sensor[3], 2) + pow(vehicle_sensor[4], 2));
    vehicle.s = vehicle_sensor[5];
    vehicle.end_s = vehicle.s + vehicle.speed * t;
    vehicle.d = vehicle_sensor[6];
    vehicle.lane = getLane(vehicle.d);

    predictions.push_back(vehicle);
  }
}

int PathPlanner::getLane(double d) {
  return static_cast<int>(d / lane_width);
}

Goal PathPlanner::getGoal() {
  Goal goal = Goal();

  const double last_s = current.s + current.speed * 2;
  vector<double> lane_speed;
  vector<double> lane_last_s;
  for (int i = 0; i < num_lanes + 2; i++) {
    if (i == 0 || i == num_lanes + 1) {
      lane_speed.push_back(0);
    } else {
      lane_speed.push_back(speed_limit);
    }
    lane_last_s.push_back(last_s);
  }
  for (const Vehicle vehicle: predictions) {
    if (vehicle.s > current.s + safe_distance && vehicle.end_s > current.end_s + safe_distance) {
      if (vehicle.s < lane_last_s[vehicle.lane + 1]) {
        lane_speed[vehicle.lane + 1] = vehicle.speed;
        lane_last_s[vehicle.lane + 1] = vehicle.s;
      }
    } else if (vehicle.s > current.s - 2 * safe_distance || vehicle.end_s > current.end_s - safe_distance) {
      lane_speed[vehicle.lane + 1] = 0;
      lane_last_s[vehicle.lane + 1] = 0;
    }
  }

  // std::cout << "Lane Speed: " << lane_speed[1] << ", " << lane_speed[2] << ", " << lane_speed[3] << std::endl;

  // Convolution lane speed for lane change, this algorithm can generate a 2 lane change behavior
  vector<double> lane_score(static_cast<unsigned long>(num_lanes));
  for (int i = 0; i < num_lanes; i++) {
    if (lane_speed[i + 1] > 0) {
      lane_score[i] = 0.1 * lane_speed[i] + lane_speed[i + 1] + 0.1 * lane_speed[i + 2];
    } else {
      lane_score[i] = 0;
    }
  }

  if (goal_lane == current.lane || goal_lane < 0) {
    const double left_score = current.lane > 0 ? lane_score[current.lane - 1] : 0;
    const double right_score = current.lane < num_lanes - 1 ? lane_score[current.lane + 1] : 0;
    const double score = lane_score[current.lane] * score_threshold;

    // std::cout << current.lane << " of Score: " << left_score << ", " << score << ", " << right_score << std::endl;
    if (left_score > score) {
      if (left_score > right_score) {
        goal_lane = current.lane - 1;
      } else {
        goal_lane = current.lane + 1;
      }
    } else if (right_score > score) {
      goal_lane = current.lane + 1;
    } else {
      goal_lane = current.lane;
    }
  }

  goal.d = lane_width * (goal_lane + 0.5);
  if (goal_lane == current.lane) {
    goal.speed = lane_speed[goal_lane + 1];
  } else {
    goal.speed = current.speed;  // Avoid AccT when changing lane
  }
  return goal;
}

double PathPlanner::getSafeDistance(double speed) {
  const double delta_speed = current.speed - speed;
  if (delta_speed > 0) {
    return delta_speed * delta_speed / max_braking_acc / 2 + safe_distance;
  } else {
    return safe_distance;
  }
}



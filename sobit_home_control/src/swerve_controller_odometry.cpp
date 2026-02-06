#include "sobit_home_control/swerve_controller_control.hpp"
#include "sobit_home_control/swerve_controller_odometry.hpp"

// Calculate Odometry
void SobitHomeOdometry::update_odom(nav_msgs::msg::Odometry &odom)
{
  nav_msgs::msg::Odometry result_odom = odom;

  // get the movement of each wheel[m]
  std::vector<double> distance_m(4);
  for (int i=0; i<4; i++) 
    distance_m[i] = distance_calculation(current_drive_pos[i] - prev_drive_pos[i]);

  // Transform to Roll, Pitch and Yaw from previous odom
  tf2::Quaternion quat_tf;
  double prev_roll, prev_pitch, prev_yaw;
  tf2::fromMsg(odom.pose.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(prev_roll, prev_pitch, prev_yaw);

  // each wheel point from robot base
  std::vector<geometry_msgs::msg::Point> wheels_point(4);
  wheels_point[0].x = wheels_point[1].x = WHEEL_X_DISTANCE / 2.;        // X position of front wheel is (+)
  wheels_point[2].x = wheels_point[3].x = WHEEL_X_DISTANCE / 2. * (-1); // X position of  back wheel is (-)
  wheels_point[0].y = wheels_point[2].y = WHEEL_Y_DISTANCE / 2.;        // Y position of  left wheel is (+)
  wheels_point[1].y = wheels_point[3].y = WHEEL_Y_DISTANCE / 2. * (-1); // Y position of right wheel is (-)

  // calculate the base_center. base_center is center in SWEVEL Motion's robot movement circle
  // 4C2 = 6 -> i:j=[0~5]
  double diff_x = 0.,diff_y = 0., diff_yaw = 0.;
  int normalize = ((wheels_point.size()*(wheels_point.size()-1)) / (2.*1.));
  for (size_t i=0; i<wheels_point.size()-1; i++) {
    for (size_t j=i+1; j<wheels_point.size(); j++) {

      double x=0., y=0., yaw=0.;
      geometry_msgs::msg::Point base_center;
      double wheel_rad_1 = current_steer_pos[i] + ((0. < distance_m[i]) ? 0. : M_PI);
      double wheel_rad_2 = current_steer_pos[j] + ((0. < distance_m[j]) ? 0. : M_PI);
      double wheel_m_1 = fabsf(distance_m[i]);
      double wheel_m_2 = fabsf(distance_m[j]);
      if (1. - fabsf(cos(wheel_rad_2 - wheel_rad_1)) < 0.001) { // near the parallel of current_steer_pos[j] and current_steer_pos[i]
        if ((fabsf(wheel_m_2 - wheel_m_1) < 0.001) && (0. < cos(wheel_rad_2 - wheel_rad_1))) { // Translational motion
          double delta = wheel_rad_2 - wheel_rad_1 - M_PI*((int)((wheel_rad_2 - wheel_rad_1)/M_PI));
          delta -= M_PI * ((int)(delta/(M_PI/2.)));
          x = (wheel_m_1 + wheel_m_2)/2. * cos(wheel_rad_1 + delta/2.);
          y = (wheel_m_1 + wheel_m_2)/2. * sin(wheel_rad_1 + delta/2.);
          yaw = 0.;
          base_center.x = base_center.y = INFINITY;
        } else {
          int ie = (0. < cos(wheel_rad_2 - wheel_rad_1)) ? -1.: 1.; //internally or externally divide
          base_center.x = (wheel_m_2*ie * wheels_point[i].x + wheel_m_1 * wheels_point[j].x) / (wheel_m_1 + wheel_m_2*ie);
          base_center.y = (wheel_m_2*ie * wheels_point[i].y + wheel_m_1 * wheels_point[j].y) / (wheel_m_1 + wheel_m_2*ie);
        }
      } else {
        double a1 = tan(wheel_rad_1 + M_PI/2.);
        double a2 = tan(wheel_rad_2 + M_PI/2.);
        base_center.x = ((a1*wheels_point[i].x - a2*wheels_point[j].x) - (wheels_point[i].y - wheels_point[j].y)) / (a1 - a2);
        base_center.y = a1 * (base_center.x - wheels_point[i].x) + wheels_point[i].y;
      }

      // 
      if ((std::isfinite(base_center.x)) || (std::isfinite(base_center.y))) {
        // 
        double dist_base_wheel_1 = sqrtf(powf((wheels_point[i].x - base_center.x), 2.) + powf((wheels_point[i].y - base_center.y), 2.));
        double dist_base_wheel_2 = sqrtf(powf((wheels_point[j].x - base_center.x), 2.) + powf((wheels_point[j].y - base_center.y), 2.));
        double pn1, pn2;
        pn1 = pn2 = 0.5;

        if (dist_base_wheel_1 < 0.001) {
          pn1 = 0;
          pn2 = 1.;
          dist_base_wheel_1 = 1.; // dummy of zero devided...
        }
        if (dist_base_wheel_2 < 0.001) {
          pn2 = 0;
          pn1 = 1.;
          dist_base_wheel_2 = 1.; // dummy of zero devided...
        }
        
        if (cos(atan2(base_center.y-wheels_point[i].y, base_center.x-wheels_point[i].x) - current_steer_pos[i] - M_PI/2.) < 0.) 
          pn1 = -1 * pn1;
        if (cos(atan2(base_center.y-wheels_point[j].y, base_center.x-wheels_point[j].x) - current_steer_pos[j] - M_PI/2.) < 0.) 
          pn2 = -1 * pn2;

        // 
        yaw = distance_m[i] * pn1 / dist_base_wheel_1 + distance_m[j] * pn2 / dist_base_wheel_2;

        // 
        x = base_center.x + (-base_center.x) * cos(yaw) - (-base_center.y) * sin(yaw);
        y = base_center.y + (-base_center.x) * sin(yaw) + (-base_center.y) * cos(yaw);

      }

      // 
      if (!std::isfinite(x)  ) x   = 0.;
      if (!std::isfinite(y)  ) y   = 0.;
      if (!std::isfinite(yaw)) yaw = 0.;

      // 
      diff_x   += x   / normalize;
      diff_y   += y   / normalize;
      diff_yaw += yaw / normalize;
    }
  }

  // Update the Odometry
  result_odom.pose.pose.position.x = odom.pose.pose.position.x + 
      diff_x * cos(prev_yaw) - diff_y * sin(prev_yaw);
  result_odom.pose.pose.position.y = odom.pose.pose.position.y + 
      diff_x * sin(prev_yaw) + diff_y * cos(prev_yaw);

  // Change quaternion
  quat_tf.setRPY(0., 0., (prev_yaw + diff_yaw));
  tf2::convert(quat_tf, result_odom.pose.pose.orientation);

  // Update the Vellocity of robot
  result_odom.twist.twist.linear.x  = diff_x   * CYCLE_FEQUENCY;
  result_odom.twist.twist.linear.y  = diff_y   * CYCLE_FEQUENCY;
  result_odom.twist.twist.angular.z = diff_yaw * CYCLE_FEQUENCY;

  odom = result_odom;
}

// Distance calculation
double SobitHomeOdometry::distance_calculation(double wheel_delta_pos) {
  return WHEEL_RADIUS * wheel_delta_pos;
}

// Pose broadcaster (Generate a pose from Odometry)
void SobitHomeOdometry::pose_broadcaster(const nav_msgs::msg::Odometry &tf_odom) {
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header          = tf_odom.header;
  transformStamped.child_frame_id  = tf_odom.child_frame_id;

  transformStamped.transform.translation.x = tf_odom.pose.pose.position.x;
  transformStamped.transform.translation.y = tf_odom.pose.pose.position.y;
  transformStamped.transform.translation.z = tf_odom.pose.pose.position.z;

  transformStamped.transform.rotation      = tf_odom.pose.pose.orientation;

  tf_broadcaster_->sendTransform(transformStamped);
}
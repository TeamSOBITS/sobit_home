#include "sobit_home_control/swerve_controller_control.hpp"

void SobitHomeControl::twist_callback(const geometry_msgs::msg::Twist::SharedPtr vel_twist)
{
  // Translational
  if (((std::fabs(vel_twist->linear.x) > 0.000) || (std::fabs(vel_twist->linear.y) > 0.000))
      && (std::fabs(vel_twist->angular.z) == 0.000))
    motion_mode = MODE::TRANSLATIONAL_MOTION_MODE;
  // Rotational
  else if ((std::fabs(vel_twist->linear.x) <= 0.001) && (std::fabs(vel_twist->linear.y) <= 0.001) && (std::fabs(vel_twist->angular.z) > 0.000))
    motion_mode = MODE::ROTATIONAL_MOTION_MODE;
  // Swivel
  else if (((std::fabs(vel_twist->linear.x) > 0.000) || (std::fabs(vel_twist->linear.y) > 0.000)) && (std::fabs(vel_twist->angular.z) > 0.000)) {
    if (std::fabs(std::sqrt(std::pow(vel_twist->linear.x, 2.0) + std::pow(vel_twist->linear.y, 2.0)) / vel_twist->angular.z) > sqrtf(powf(WHEEL_X_DISTANCE/2., 2.) + powf(WHEEL_Y_DISTANCE/2., 2.)))
         motion_mode = MODE::SWIVEL_MOTION_MODE;
    else motion_mode = MODE::ROTATIONAL_MOTION_MODE; // base_center point is inner of robot
  }
  // Stop
  else motion_mode = MODE::STOP_MOTION_MODE;


  switch (motion_mode) {
    // Stop motion
    case STOP_MOTION_MODE: {
      goal_drive_vel[0] = goal_drive_vel[1] = goal_drive_vel[2] = goal_drive_vel[3] = 0.;
      break;
    }

    // Translational motion
    case TRANSLATIONAL_MOTION_MODE: {
      // Goal position calculation
      double goal_rad = atan2(vel_twist->linear.y, vel_twist->linear.x);

      // Goal velocity calculation
      double vel_rads = sqrtf(powf(vel_twist->linear.x, 2.) + powf(vel_twist->linear.y, 2.)) / WHEEL_RADIUS; // vel_twist[m/s] to vel_rads[rad/s]

      goal_steer_pos[0] = goal_steer_pos[1] = goal_steer_pos[2] = goal_steer_pos[3] = goal_rad;

      // Direction of wheel rotation
      if (vel_rads > DRIVE_MAX_VEL) vel_rads = DRIVE_MAX_VEL;
  
      goal_drive_vel[0] = goal_drive_vel[1] = goal_drive_vel[2] = goal_drive_vel[3] = vel_rads;
      break;
    }

    // Rotational motion
    case ROTATIONAL_MOTION_MODE: {
      // Goal velocity calculation
      double vel_ms   = vel_twist->angular.z * sqrtf(powf(WHEEL_X_DISTANCE/2., 2.) + powf(WHEEL_Y_DISTANCE/2., 2.));
      double vel_rads = vel_ms / WHEEL_RADIUS;

      // Goal angle calculation
      goal_steer_pos[0] = atan2( WHEEL_Y_DISTANCE/2.,  WHEEL_X_DISTANCE/2.) + M_PI/2.;
      goal_steer_pos[1] = atan2(-WHEEL_Y_DISTANCE/2.,  WHEEL_X_DISTANCE/2.) + M_PI/2.;
      goal_steer_pos[2] = atan2( WHEEL_Y_DISTANCE/2., -WHEEL_X_DISTANCE/2.) + M_PI/2.;
      goal_steer_pos[3] = atan2(-WHEEL_Y_DISTANCE/2., -WHEEL_X_DISTANCE/2.) + M_PI/2.;

      // Velocity of wheel
      if (fabsf(vel_rads) > DRIVE_MAX_VEL) vel_rads = DRIVE_MAX_VEL * (vel_rads/fabsf(vel_rads));
      
      goal_drive_vel[0] = goal_drive_vel[1] = goal_drive_vel[2] = goal_drive_vel[3] = vel_rads;
      break;
    }

    // Swivel motion
    case SWIVEL_MOTION_MODE: {
      double base_vel = sqrtf(powf(vel_twist->linear.x, 2.) + powf(vel_twist->linear.y, 2.));
      double r = base_vel / fabsf(vel_twist->angular.z);
      double base_angle = atan2(vel_twist->linear.y , vel_twist->linear.x);
      int angle_pn = (0. < vel_twist->angular.z) ? 1 : -1;

      geometry_msgs::msg::Point base_center;
      base_center.x = r * cosf(base_angle + (M_PI/2.) * angle_pn);
      base_center.y = r * sinf(base_angle + (M_PI/2.) * angle_pn);

      // each wheel point from robot base
      geometry_msgs::msg::Point wheel_point[4]; // {"FL", "FR", "BL", "BR"}
      wheel_point[0].x = wheel_point[1].x = WHEEL_X_DISTANCE / 2.;        // X position of front wheel is (+)
      wheel_point[2].x = wheel_point[3].x = WHEEL_X_DISTANCE / 2. * (-1); // X position of  back wheel is (-)
      wheel_point[0].y = wheel_point[2].y = WHEEL_Y_DISTANCE / 2.;        // Y position of  left wheel is (+)
      wheel_point[1].y = wheel_point[3].y = WHEEL_Y_DISTANCE / 2. * (-1); // Y position of right wheel is (-)

      double base_vel_rads = base_vel / WHEEL_RADIUS;

      // calculate the direction of each wheel (|direction| > 2PI is okay. )
      // And, calculate the velocity of each wheel ([rad/s])
      for (int i=0; i<4; i++) {
        goal_steer_pos[i] = atan2(wheel_point[i].y - base_center.y, wheel_point[i].x - base_center.x) + M_PI/2.*angle_pn;
        goal_drive_vel[i] = base_vel_rads * sqrtf(powf(wheel_point[i].x - base_center.x, 2.) + powf(wheel_point[i].y - base_center.y, 2.)) / r;
      }

      break;
    }

    // Other motion
    default: {
      goal_drive_vel[0] = goal_drive_vel[1] = goal_drive_vel[2] = goal_drive_vel[3] = 0.;
      break;
    }
  }
  
  // Normalize steering angles into [0, 2π)
  for (int i=0; i<4; i++) 
    goal_steer_pos[i] = goal_steer_pos[i] - (2*M_PI) * ((int)(goal_steer_pos[i] / (2*M_PI)));

  // Wrap angles to [-π, π]
  for (int i=0; i<4; i++) {
    if (M_PI < fabsf(goal_steer_pos[i])) 
      goal_steer_pos[i] -= 2*M_PI*goal_steer_pos[i]/fabsf(goal_steer_pos[i]);
  }

  for (int i=0; i<4; i++) {
    // Steering movable in [-π, π]
    if ((fabsf(goal_steer_pos[i] - M_PI*goal_steer_pos[i]/fabsf(goal_steer_pos[i]) - current_steer_pos[i]) < fabsf(goal_steer_pos[i] - current_steer_pos[i])) && 
        (fabsf(goal_steer_pos[i] - M_PI*goal_steer_pos[i]/fabsf(goal_steer_pos[i]) - current_steer_pos[i]) <M_PI)) {
      goal_steer_pos[i] -= M_PI*goal_steer_pos[i]/fabsf(goal_steer_pos[i]);
      goal_drive_vel[i] *= -1;
    }
    // Steering movable in [-π/2, π/2]
    // if (M_PI/2. < fabsf(goal_steer_pos[i])) {
    //   goal_steer_pos[i] -= M_PI*goal_steer_pos[i]/fabsf(goal_steer_pos[i]);
    //   goal_drive_vel[i] *= -1;
    // }
  }
}

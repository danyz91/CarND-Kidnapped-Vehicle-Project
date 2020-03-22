/*
 * particle_utils.h
 *
 *  Created on: Mar 21, 2020
 *      Author: danyz
 */

#ifndef SRC_PARTICLE_UTILS_H_
#define SRC_PARTICLE_UTILS_H_


#include <math.h>
#include <vector>


/**
 * Compute evolution from location (curr_x, curr_y, curr_theta)
 * with bicycle model as motion model. The evolution is computed
 * according to dynamic params (velocity, yaw_rate and delta_t)
 * @param curr_x Initial x position [m]
 * @param curr_y Initial y position [m]
 * @param curr_theta Initial yaw [rad]
 * @param velocity Velocity of car from t to t+1 [m/s]
 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
 * @param delta_t Time between time step t and t+1 in measurements [s]
 */
inline std::vector<double> runBicycleModel(double curr_x, double curr_y,
                                    double curr_theta, double velocity,
                                    double yaw_rate, double delta_t) {
  std::vector<double> result(3);

  double new_x, new_y, new_theta;

  if (yaw_rate != 0) {
    double theta_future = yaw_rate * delta_t;
    double speed_fract = velocity/yaw_rate;

    new_x = curr_x + speed_fract *
        (sin(curr_theta + theta_future) - sin(curr_theta));
    new_y = curr_y + speed_fract *
        (cos(curr_theta) - cos(curr_theta + theta_future));
    new_theta = curr_theta + theta_future;
  } else {
    new_x = curr_x + velocity * delta_t * cos(curr_theta);
    new_y = curr_y + velocity * delta_t * sin(curr_theta);
    new_theta = curr_theta;
  }

  result.push_back(new_x);
  result.push_back(new_y);
  result.push_back(new_theta);

  return result;
}

/**
 * Compute multi variate gaussian density function value.
 * This function tells us how likely a set of landmark measurements
 * is given our predicted state of the car and the assumption that
 * the sensors have gaussian noise.
 * @param x_obs Testing location x
 * @param y_obs Testing location y
 * @param mu_x Gaussian mean over x, associated landmark position x
 * @param mu_y Gaussian mean over y, associated landmark position y
 * @param sig_x Uncertainty over x range
 * @param sig_y Unvertainty over y range
 */
inline double multiv_prob(double x_obs, double y_obs,
                          double mu_x, double mu_y,
                          double sig_x, double sig_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
              + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);

  return weight;
}


/**
 * Compute the homogeneous transformation of point (obs_x, obs_y)
 * in the new reference frame located at (origin_x, origin_y) and
 * with a rotation of theta.
 * @param obs_x Test point x
 * @param obs_y Test point y
 * @param origin_x X coordinate of new origin
 * @param origin_y Y coordinate of new origin
 * @param theta Rotation angle to match the reference frames
 */
inline std::vector<double> getTransformation(double obs_x, double obs_y,
                                             double origin_x, double origin_y,
                                             double theta) {

  std::vector<double> result;

  double x_conv = origin_x + cos(theta) * obs_x - sin(theta) * obs_y;
  double y_conv = origin_y + sin(theta) * obs_x + cos(theta) * obs_x;

  result.push_back(x_conv);
  result.push_back(y_conv);

  return result;
}




#endif  // SRC_PARTICLE_UTILS_H_

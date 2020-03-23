/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  if (is_initialized) {
    return;
  }

  num_particles = NUM_PARTICLES;  // Set the number of particles

  // Create and set standard deviations for x, y, and theta
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // These lines create a normal (Gaussian) distribution for x,y and theta
  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);

  // Init all particles
  for (int i = 0; i < num_particles; i++) {
    Particle curr_particle;
    curr_particle.id = i;
    curr_particle.x = dist_x(gen);
    curr_particle.y = dist_y(gen);
    curr_particle.theta = dist_theta(gen);
    curr_particle.weight = 1.0;
    particles.push_back(curr_particle);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  for (int i = 0; i < num_particles; i++) {
    std::vector<double> predicted_pos = runBicycleModel(particles[i].x,
                                                        particles[i].y,
                                                        particles[i].theta,
                                                        velocity, yaw_rate,
                                                        delta_t);

    // Position and Yaw update according to bicycleModel
    particles[i].x = predicted_pos[0];
    particles[i].y = predicted_pos[1];
    particles[i].theta = predicted_pos[2];

    // Noise setup for current particle
    std::normal_distribution<double> dist_x(0, std_x);
    std::normal_distribution<double> dist_y(0, std_y);
    std::normal_distribution<double> dist_theta(0, std_theta);

    // Noise adding
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations) {
  /**
   * Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  // If no predicted landmark is present, no association is made
  if (predicted.size() == 0) {
    return;
  }

  // For each observation, find the nearest landmark
  for (int i = 0; i < observations.size(); i++) {
    double min_dist = std::numeric_limits<double>::max();
    double min_dist_id = -1.0;

    for (int j = 0; j < predicted.size(); j++) {
      LandmarkObs curr_predicted_measurement = predicted[j];

      double curr_dist = dist(curr_predicted_measurement.x,
                              curr_predicted_measurement.y, observations[i].x,
                              observations[i].y);

      if (curr_dist < min_dist) {
        min_dist = curr_dist;
        min_dist_id = curr_predicted_measurement.id;
      }
    }

    // Associate curr observation with nearest landmark
    observations[i].id = min_dist_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  double sensor_range_squared = sensor_range * sensor_range;

  // ########################################################
  // For each particle
  // ########################################################
  for (int i = 0; i < num_particles; i++) {
    Particle &curr_particle = particles[i];
    curr_particle.weight = 1.0;

    // ########################################################
    // Observations transformation in particle reference frame
    // ########################################################
    std::vector<LandmarkObs> trans_observations;

    for (int j = 0; j < observations.size(); j++) {
      std::vector<double> obs_transformed = getTransformation(
          observations[j].x, observations[j].y, curr_particle.x,
          curr_particle.y, curr_particle.theta);
      trans_observations.push_back(LandmarkObs{observations[j].id,
                                    obs_transformed[0], obs_transformed[1]});
    }

    // ########################################################
    // Select landmarks within sensor range i.e. predicted measurements
    // ########################################################
    std::vector<LandmarkObs> visible_landmarks;
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      Map::single_landmark_s curr_landmark = map_landmarks.landmark_list[j];
      double curr_dist = dist(curr_particle.x, curr_particle.y,
                              curr_landmark.x_f, curr_landmark.y_f);
      if (curr_dist <= sensor_range_squared) {
        visible_landmarks.push_back(LandmarkObs{curr_landmark.id_i,
                                      curr_landmark.x_f, curr_landmark.y_f});
      }
    }

    // ########################################################
    // Associate transformed observaiton to predicted landmarks
    // ########################################################
    dataAssociation(visible_landmarks, trans_observations);

    // Weight reset
    curr_particle.weight = 1.0;

#ifdef DEBUG_INFO
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
#endif
    // ########################################################
    // Weight update
    // ########################################################
    for (int j = 0; j < trans_observations.size(); j++) {
      LandmarkObs curr_trans_observation = trans_observations[j];

      int associated_map_landmark_id = curr_trans_observation.id;
      // Find back predicted landmark to which transformed observation
      // has been associated
      auto it = find_if(visible_landmarks.begin(), visible_landmarks.end(),
                        [associated_map_landmark_id](LandmarkObs curr) {
                          return curr.id == associated_map_landmark_id;
                        });

      double curr_weight = 1.0;

      if (it != visible_landmarks.end()) {
        LandmarkObs curr_land =
            visible_landmarks[it - visible_landmarks.begin()];

        curr_weight = multiv_prob(curr_trans_observation.x,
                                  curr_trans_observation.y, curr_land.x,
                                  curr_land.y, std_landmark[0],
                                  std_landmark[1]);

        if (curr_weight < std::numeric_limits<double>::min()) {
          curr_weight = std::numeric_limits<double>::min();
        }
        // Populate debug information
#ifdef DEBUG_INFO
        associations.push_back(associated_map_landmark_id);
        sense_x.push_back(curr_land.x);
        sense_y.push_back(curr_land.y);
#endif
      }

      curr_particle.weight *= curr_weight;
    }
#ifdef DEBUG_INFO
    SetAssociations(curr_particle, associations, sense_x, sense_y);
#endif
  }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  std::default_random_engine gen;

  // list initializer with starting weights
  std::vector<double> weights;

  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  std::discrete_distribution<> discrete_dist(weights.begin(), weights.end());

  std::vector<Particle> new_particles;
  // Resampling loop
  for (int i = 0; i < num_particles; i++) {
    int selected_index = discrete_dist(gen);
    new_particles.push_back(particles[selected_index]);
  }

  weights.clear();
  // Assign new particles to filter
  particles = new_particles;

}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

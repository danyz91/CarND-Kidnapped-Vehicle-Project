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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = NUM_PARTICLES;  // Set the number of particles
  std::default_random_engine gen;

  // Create and set standard deviations for x, y, and theta
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  // These lines create a normal (Gaussian) distribution for x,y and theta
  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);


  for (int i=0; i < num_particles; i++) {
    Particle curr_particle;
    curr_particle.id = i;
    curr_particle.x = dist_x(gen);
    curr_particle.y = dist_y(gen);
    curr_particle.theta = dist_theta(gen);
    curr_particle.weight = 1.0;
    particles.push_back(curr_particle);
  }

  is_initialized = true;

  std::cout << "Initialized!"<< std::endl;
  for (Particle p : particles) {
    std::cout << p.x << " , " << p.y << " , " << p.theta << std::endl;
  }

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

  std::default_random_engine gen;
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  std::vector<double> predicted_pos;

  for (int i=0; i < num_particles; i++) {
    Particle curr_particle = particles[i];
    predicted_pos = runBicycleModel(curr_particle.x, curr_particle.y,
                                            curr_particle.theta, velocity,
                                            yaw_rate, delta_t);

    // Position and Yaw update according to bicycleModel
    curr_particle.x = predicted_pos[0];
    curr_particle.y = predicted_pos[1];
    curr_particle.theta = predicted_pos[2];

    // Noise setup for current particle
    std::normal_distribution<double> dist_x(curr_particle.x, std_x);
    std::normal_distribution<double> dist_y(curr_particle.y, std_y);
    std::normal_distribution<double> dist_theta(curr_particle.theta, std_theta);

    // Noise adding
    curr_particle.x += dist_x(gen);
    curr_particle.y += dist_y(gen);
    curr_particle.theta += dist_theta(gen);

    predicted_pos.clear();
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  double min_dist = std::numeric_limits<double>::max();

  for (auto curr_pred : predicted) {
    for (auto& obs : observations) {
      double curr_dist = dist(curr_pred.x, curr_pred.y, obs.x, obs.y);
      if (curr_dist < min_dist)  {
        min_dist = curr_dist;
        obs.id = curr_pred.id;
      }
    }
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO(danyz91): Update the weights of each particle using a mult-variate Gaussian
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

  std::vector<double> obs_transformed;
  std::vector<LandmarkObs> trans_observations;
  for (int i=0; i < num_particles; i++) {

    Particle curr_particle = particles[i];

    // Observations tranformation in particle reference frame
    for (int j=0; j < observations.size(); j++) {
      obs_transformed = getTransformation(observations[j].x,
                                                 observations[j].y,
                                                 curr_particle.x,
                                                 curr_particle.y,
                                                 curr_particle.theta);


      LandmarkObs curr_obs;
      curr_obs.id = observations[j].id;
      curr_obs.x = obs_transformed[0];
      curr_obs.y = obs_transformed[1];

      trans_observations.push_back(curr_obs);

      obs_transformed.clear();
    }

    // associate curr particle to a landmark (convert in global ref frame)

    std::vector<LandmarkObs> predicted;
    for (int j=0; j < map_landmarks.landmark_list.size(); j++) {
      Map::single_landmark_s curr_landmark = map_landmarks.landmark_list[j];
      double curr_dist = dist(curr_particle.x, curr_particle.y,
                              curr_landmark.x_f, curr_landmark.y_f);

      if (curr_dist <= sensor_range) {
        LandmarkObs curr_pred_obs;
        curr_pred_obs.id = curr_landmark.id_i;
        curr_pred_obs.x = curr_landmark.x_f;
        curr_pred_obs.y = curr_landmark.y_f;
        predicted.push_back(curr_pred_obs);
      }
    }

    dataAssociation(predicted, trans_observations);

    for (int j=0; j < trans_observations.size(); j++) {
      LandmarkObs curr_obs = trans_observations[j];

      auto it = std::find_if(predicted.begin(), predicted.end(),
                             [curr_obs](const LandmarkObs& curr){
                             return curr.id == curr_obs.id;});

      LandmarkObs map_landmark = predicted[it - predicted.begin()];

      curr_particle.weight *= multiv_prob(curr_obs.x, curr_obs.y, map_landmark.x,
                                         map_landmark.y, std_landmark[0],
                                         std_landmark[1]);

      std::cout<<"multi var : "<<multiv_prob(curr_obs.x, curr_obs.y, map_landmark.x,
                                             map_landmark.y, std_landmark[0],
                                             std_landmark[1]) << std::endl;

      std::cout<<" weight "<<curr_particle.weight<<std::endl;
    }

    trans_observations.clear();
    predicted.clear();
  }


  //for (Particle p : particles) {
  //    std::cout << p.x << " , " << p.y << " , " << p.theta << " , " << p.weight << std::endl;
  //  }

}

void ParticleFilter::resample() {
  /**
   * TODO(danyz91): Resample particles with replacement with probability proportional
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  //std::default_random_engine gen;
  // list initializer with starting weights
  //std::discrete_distribution<> d({40, 10, 10, 40});


}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
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
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

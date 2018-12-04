/*
 * particle_filter.cpp
 *
 * Created on Dec 12, 2016 by Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"


void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // TODO Has to be implemented
  // Set the number of particles. Initialize all particles to first position (based on estimates of
  // x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  num_particles_ = 1000;

  std::default_random_engine gen{};

  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  std::normal_distribution<double> dist_x{x, std_x};
  std::normal_distribution<double> dist_y{y, std_y};
  std::normal_distribution<double> dist_theta{theta, std_theta};

  double sample_x, sample_y, sample_theta;

  for (int i = 0; i < num_particles_; i++) {

    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);

    Particle p;
    p.id = i;
    p.x = sample_x;
    p.y = sample_y;
    p.theta = sample_theta;
    p.weight = 1.;

    particles.push_back(p);
  }

  is_initialized_ = true;

}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // TODO Has to be implemented
  // Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  // http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  // http://www.cplusplus.com/reference/random/default_random_engine/

  std::default_random_engine gen{};

  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  std::normal_distribution<double> dist_x{0, std_x};
  std::normal_distribution<double> dist_y{0, std_y};
  std::normal_distribution<double> dist_theta{0, std_theta};

  for (Particle& p : particles) {

    double vt = velocity / yaw_rate;
    double ydt = yaw_rate * delta_t;

    double inc_x = vt * (sin(p.theta + ydt) - sin(p.theta));
    double inc_y = vt * (cos(p.theta) - cos(p.theta + ydt));
    double inc_theta = p.theta;

    double noise_x = dist_x(gen);
    double noise_y = dist_y(gen);
    double noise_theta = dist_theta(gen);

    p.x += (inc_x + noise_x);
    p.y += (inc_y + noise_y);
    p.theta += (inc_theta + noise_theta);

  }

}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

  // TODO: Has to be implemented
  // Find the predicted measurement that is closest to each observed measurement and assign the
  // observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
  // implement this method and use it as a helper during the updateWeights phase.

  // TODO What are `predicted` and how to use them?

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs>& observations, const Map& map_landmarks) {
  // TODO: Has to be implemented
  // Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  // more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  // according to the MAP'S coordinate system. You will need to transform between the two systems.
  // Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  // The following is a good resource for the theory:
  // https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  // and the following is a good resource for the actual equation to implement (look at equation 3.33)
  // http://planning.cs.uiuc.edu/node99.html

  for (Particle& p : particles) {

    std::vector<LandmarkObs> predicted;
    for (const LandmarkObs& obs : observations) {

      LandmarkObs obs_p{};
      obs_p.id = obs.id;
      obs_p.x = cos(p.theta) * obs.x - sin(p.theta) * obs.y + p.x;
      obs_p.y = sin(p.theta) * obs.x + cos(p.theta) * obs.y + p.y;

      predicted.push_back(obs_p);

    }


  }




}


void ParticleFilter::resample() {

  // TODO: Has to be implemented
  // Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  // http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}


Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                         const std::vector<double>& sense_x, const std::vector<double>& sense_y) {

  // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

}


std::string ParticleFilter::getAssociations(Particle best) {
  std::vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}


std::string ParticleFilter::getSenseX(Particle best) {

  std::vector<double> v = best.sense_x;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;

}


std::string ParticleFilter::getSenseY(Particle best) {

  std::vector<double> v = best.sense_y;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;

}

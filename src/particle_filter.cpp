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

std::vector<double> simple_predict(const std::vector<double>& pose, double velocity, double yaw_rate, double delta_t) {

  double x = pose[0];
  double y = pose[1];
  double theta = pose[2];

  double inc_x;
  double inc_y;
  double inc_theta;

  if (fabs(yaw_rate) >= 0.001) {

    double vt = velocity / yaw_rate;
    double ydt = yaw_rate * delta_t;

    inc_x = vt * (sin(theta + ydt) - sin(theta));
    inc_y = vt * (cos(theta) - cos(theta + ydt));
    inc_theta = ydt;

  } else {

    double vdt = velocity * delta_t;

    inc_x = vdt * cos(theta);
    inc_y = vdt * sin(theta);
    inc_theta = 0;

  }

  x += inc_x;
  y += inc_y;
  theta += inc_theta;

  return std::vector<double>{x, y, theta};

}

std::pair<double, double > transform(const LandmarkObs& obs, const Particle& p) {

  double x = cos(p.theta) * obs.x - sin(p.theta) * obs.y + p.x;
  double y = sin(p.theta) * obs.x + cos(p.theta) * obs.y + p.y;

  return std::pair<double, double>{x, y};

}

int findNearestLandmark(const double& x_p, const double& y_p, const Map& map) {

  std::vector<double> distances;
  for (auto& landmark : map.landmark_list) {

    double distance = dist(x_p, y_p, landmark.x_f, landmark.y_f);
    distances.push_back(distance);

  }

  auto nearest = std::min_element(distances.begin(), distances.end()) - distances.begin();

  return (int)nearest;

}


double gaussian2D(double x, double y, double mu_x, double mu_y, double std_x, double std_y) {

  long double a = 1. / (2. * M_PI * std_x * std_y);

  long double diff_x = x - mu_x;
  long double diff_y = y - mu_y;

  long double b1 = (diff_x * diff_x) / (2 * std_x * std_x);
  long double b2 = (diff_y * diff_y) / (2 * std_y * std_y);

  long double prob = a * std::exp(-(b1 + b2));

  return (double)prob;

}


void updateParticleWeight(Particle* p, const Map& map, double std_landmark_x, double std_landmark_y) {

  p->weight = 1.;

  for (unsigned int i = 0; i < p->associations.size(); i++) {

    int landmark_idx = p->associations[i];
    auto landmark = map.landmark_list[landmark_idx];

    double x = p->sense_x[i];
    double y = p->sense_y[i];

    double prob = gaussian2D(x, y, landmark.x_f, landmark.y_f, std_landmark_x, std_landmark_y);

    p->weight *= prob;

    //std::cout << prob << ", " << p->weight << "\n";

  }

  //std::cout << "=====\n";

}

double newParticleWeight(
    const std::vector<int>& indices,
    const std::vector<std::pair<double, double>>& sense,
    const Map& map,
    double std_landmark_x,
    double std_landmark_y
    ) {

  double weight = 1.;

  for (unsigned int i = 0; i < indices.size(); i++) {

    int landmark_idx = indices[i];
    auto landmark = map.landmark_list[landmark_idx];

    double x = sense[i].first;
    double y = sense[i].second;

    double prob = gaussian2D(x, y, landmark.x_f, landmark.y_f, std_landmark_x, std_landmark_y);

    weight *= prob;
  }

  return weight;

}


void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // Set the number of particles. Initialize all particles to first position (based on estimates of
  // x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  num_particles_ = 100;

  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  std::normal_distribution<double> dist_x{x, std_x};
  std::normal_distribution<double> dist_y{y, std_y};
  std::normal_distribution<double> dist_theta{theta, std_theta};

  double sample_x, sample_y, sample_theta;

  for (int i = 0; i < num_particles_; i++) {

    sample_x = dist_x(random_engine_);
    sample_y = dist_y(random_engine_);
    sample_theta = dist_theta(random_engine_);

    Particle p;
    p.id = i;
    p.x = sample_x;
    p.y = sample_y;
    p.theta = sample_theta;
    p.weight = 1.;

    particles.push_back(p);

  }

  is_initialized_ = true;

  normalize();

}


void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  // http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  // http://www.cplusplus.com/reference/random/default_random_engine/

  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  std::normal_distribution<double> dist_x{0, std_x};
  std::normal_distribution<double> dist_y{0, std_y};
  std::normal_distribution<double> dist_theta{0, std_theta};

  for (Particle& p : particles) {

    auto current_pose = std::vector<double>{p.x, p.y, p.theta};
    auto predicted_pose = simple_predict(current_pose,  velocity, yaw_rate, delta_t);

    double noise_x = dist_x(random_engine_);
    double noise_y = dist_y(random_engine_);
    double noise_theta = dist_theta(random_engine_);

    p.x = predicted_pose[0] + noise_x;
    p.y = predicted_pose[1] + noise_y;
    p.theta = predicted_pose[2] + noise_theta;

  }

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs>& observations, const Map& map_landmarks) {

  // Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  // more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  // according to the MAP'S coordinate system. You will need to transform between the two systems.
  // Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  // The following is a good resource for the theory:
  // https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  // and the following is a good resource for the actual equation to implement (look at equation 3.33)
  // http://planning.cs.uiuc.edu/node99.html

//  for (Particle& p : particles) {
//
//    p.sense_x.clear();
//    p.sense_y.clear();
//    p.associations.clear();
//
//    for (const LandmarkObs& obs : observations) {
//
//      auto obs_t = transform(obs, p);
//
//      p.sense_x.push_back(obs_t.first);
//      p.sense_y.push_back(obs_t.second);
//
//      int assoc = findNearestLandmark(obs_t.first, obs_t.second, map_landmarks);
//      p.associations.push_back(assoc);
//
//    }
//
//    updateParticleWeight(&p, map_landmarks, std_landmark[0], std_landmark[1]);
//
//  }
//
//  normalize();

  for (Particle& p : particles) {

    std::vector<int> indices;
    std::vector<std::pair<double, double>> sense;

    for (const LandmarkObs& obs : observations) {

      auto obs_t = transform(obs, p);
      sense.push_back(obs_t);

      int idx_nearest = findNearestLandmark(obs_t.first, obs_t.second, map_landmarks);
      indices.push_back(idx_nearest);

    }

    double w = newParticleWeight(indices, sense, map_landmarks, std_landmark[0], std_landmark[1]);
    p.weight = w;

  }

}

void ParticleFilter::resample() {

  // Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  // http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  std::vector<double> current_weights;
  for (const Particle& p : particles) {
    current_weights.push_back(p.weight);
  }

  std::discrete_distribution<int> distrib{current_weights.begin(), current_weights.end()};

  std::vector<Particle> resampled_particles;
  int idx;
  for (unsigned int i = 0; i < particles.size(); i++) {

    idx = distrib(random_engine_);

    Particle p{particles[idx]};

    resampled_particles.push_back(p);

  }

  particles = resampled_particles;

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

void ParticleFilter::normalize() {

  double w_sum = 0.;
  for (Particle& p : particles) {
    w_sum += p.weight;
  }

  for (Particle& p : particles) {
    p.weight /= w_sum;
  }

}


void ParticleFilter::printWeights() {
  for (Particle& p : particles) {
    std::cout << "(" << p.x << "," << p.y << ") " << p.weight << "\n";
  }
}

/*
 * particle_filter.h
 *
 * 2D particle filter class.
 * Created on Dec 12, 2016 by Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"
#include <random>

struct Particle {

  int id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;

};

std::vector<double> simplePredict(const std::vector<double>& pose, double velocity, double yaw_rate, double delta_t);

std::pair<double, double > transform(const LandmarkObs& obs, const Particle& p);

int findNearestLandmark(const double& x_p, const double& y_p, const Map& map);

double gaussian2D(double x, double y, double mu_x, double mu_y, double std_x, double std_y);

void updateParticleWeight(Particle* p, const Map& map, double std_landmark_x, double std_landmark_y);

std::vector<double> observationsProbabilities(
    const std::vector<int>& indices,
    const std::vector<std::pair<double, double>>& sense,
    const Map& map,
    double std_landmark_x,
    double std_landmark_y
);

double newParticleWeight(const std::vector<double>& obs_probabilites);

class ParticleFilter {

private:

  // Number of particles to draw
  int num_particles_;

  // Flag, if filter is initialized
  bool is_initialized_;

  // Vector of weights of all particles
  std::vector<double> weights_;

  std::default_random_engine random_engine_;

  void normalize();
  void printWeights();

public:

  // Set of current particles
  std::vector<Particle> particles;

  // Constructor
  // @param num_particles_ Number of particles
  ParticleFilter() : num_particles_(0), is_initialized_(false) {
    std::random_device rd{};
    random_engine_ = std::default_random_engine{rd()};
  }

  // Destructor
  ~ParticleFilter() = default;

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
   *   standard deviation of yaw [rad]]
   */
  void init(double x, double y, double theta, double std[]);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
   *   standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood of the
   *   observed measurements.
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs>& observations,
                     const Map& map_landmarks);

  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void resample();

  std::string getAssociations(Particle best);

  std::string getSenseX(Particle best);

  std::string getSenseY(Particle best);

  /**
  * initialized Returns whether particle filter is initialized yet or not.
  */
  const bool initialized() const {
    return is_initialized_;
  }

};


#endif /* PARTICLE_FILTER_H_ */

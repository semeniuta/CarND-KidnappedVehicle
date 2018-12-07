#include <iostream>
#include <vector>
#include <math.h>
#include "helper_functions.h"
#include "map"
#include "particle_filter.h"

int main() {


  Map landmarks{};
  int idx = 0;
  for (const auto& pos : std::vector<std::pair<float, float>>{{5, 3}, {2, 1}, {6, 1}, {7, 4}, {4, 7}}) {
    landmarks.landmark_list.push_back(Map::single_landmark_s{idx, pos.first, pos.second});
    idx++;
  }

  idx = 0;
  std::vector<LandmarkObs> observations;
  for (const auto &obs : std::vector<std::pair<double, double>>{{2, 2}, {3, -2}, {0, -4}})
  {
    observations.push_back(LandmarkObs{idx, obs.first, obs.second});
    idx++;
  }

  Particle p;
  p.x = 4;
  p.y = 5;
  p.theta = -M_PI / 2;

  std::cout << "Transformed observations:\n";

  for (const LandmarkObs& obs : observations) {

    auto obs_t = transform(obs, p);

    int nearest = findNearestLandmark(obs_t.first, obs_t.second, landmarks);

    p.sense_x.push_back(obs_t.first);
    p.sense_y.push_back(obs_t.second);
    p.associations.push_back(nearest);

    std::cout << obs_t.first << ", " << obs_t.second << "->" << nearest << std::endl;
  }

  updateParticleWeight(&p, landmarks, 0.3, 0.3);

  // Expected probabilities of observations:
  // 6.84e-03
  // 6.84e-03
  // 9.83e-49
  // Particle weight: 4.60e-53

  // ========

  double dt = 0.1;
  double v = 110;
  double yawd = M_PI / 8;
  auto before_predict_pose = std::vector<double>{102, 65, 5 * M_PI / 8};

  auto predicted = simple_predict(before_predict_pose, v, yawd, dt);

  std::cout << "Predicted:\n";
  for (const double& val : predicted) {
    std::cout << val << "\n";
  }

  // Expected values of x, y, theta:
  // 97.59204608, 75.07741997,  2.00276532

}
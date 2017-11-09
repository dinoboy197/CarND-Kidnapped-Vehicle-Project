/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Set the number of particles. Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.

  num_particles = 100;
  std::normal_distribution<double> dist_x(0, std[0]);
  std::normal_distribution<double> dist_y(0, std[1]);
  std::normal_distribution<double> dist_theta(0, std[2]);

  for (int i = 0; i < num_particles; ++i) {
    Particle p = { };
    p.weight = 1;
    p.x = x + dist_x(gen);
    p.y = y + dist_y(gen);
    p.theta = theta + dist_theta(gen);
    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // Add measurements to each particle and add random Gaussian noise.
  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);

  for (auto &p : particles) {
    if (fabs(yaw_rate) >= 0.0001) {
      p.x += (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
      p.theta += yaw_rate * delta_t;
    } else {
      p.x += delta_t * velocity * cos(p.theta);
      p.y += delta_t * velocity * sin(p.theta);
    }

    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // Find the predicted measurement that is closest to each observed measurement and assign the
  //   observed measurement to this particular landmark.

  for (auto &observation : observations) {
    auto shortest_distance_squared = std::numeric_limits<double>::max();
    auto shortest_obs = 0;

    for (const auto & p : predicted) {
      // assign the id of the matching landmark in the map to the observation
      auto distance_squared = std::pow(p.x - observation.x, 2) + std::pow(p.y - observation.y, 2);
      if (distance_squared < shortest_distance_squared) {
        shortest_distance_squared = distance_squared;
        shortest_obs = p.id;
      }
    }
    observation.id = shortest_obs;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  // Update the weights of each particle using a multi-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html

  auto normalizer = 2 * M_PI * std_landmark[0] * std_landmark[1];
  auto x_normalizer = 2 * std::pow(std_landmark[0], 2);
  auto y_normalizer = 2 * std::pow(std_landmark[1], 2);

  // update weights for each particle
  for (auto &particle : particles) {
    // find landmarks within sensor range of the particle
    std::vector<LandmarkObs> landmarks;
    for (const auto &landmark : map_landmarks.landmark_list) {
      // only select landmarks that are within sensor range
      if (std::sqrt(std::pow(landmark.x_f - particle.x, 2) + std::pow(landmark.y_f - particle.y, 2)) < sensor_range) {
        LandmarkObs l = { };
        l.id = landmark.id_i;
        l.x = landmark.x_f;
        l.y = landmark.y_f;
        landmarks.push_back(l);
      }
    }

    // transform each observation marker from the vehicle's coordinates to the map's coordinates, with respect to the particle
    std::vector<LandmarkObs> obs_in_map_coords;
    for (const auto &observation : observations) {
      LandmarkObs ob_in_map_coords = { };
      ob_in_map_coords.x = particle.x + cos(particle.theta) * observation.x - sin(particle.theta) * observation.y;
      ob_in_map_coords.y = particle.y + sin(particle.theta) * observation.x + cos(particle.theta) * observation.y;
      obs_in_map_coords.push_back(ob_in_map_coords);
    }

    // associate the closest landmark to each observation
    dataAssociation(landmarks, obs_in_map_coords);

    // update weights for particles
    particle.weight = 1.0;
    for (const auto &ob : obs_in_map_coords) {
      std::vector<LandmarkObs>::iterator nearest_landmark_iter = std::find_if(
          landmarks.begin(), landmarks.end(), [&ob](const LandmarkObs& x) {return x.id == ob.id;});
      particle.weight *= std::exp(
          -(std::pow(ob.x - nearest_landmark_iter->x, 2) / x_normalizer
              + std::pow(ob.y - nearest_landmark_iter->y, 2) / y_normalizer)) / normalizer;
    }
  }
}

void ParticleFilter::resample() {
  // Resample particles with replacement with probability proportional to their weight.
  // use a sampling wheel.

  std::vector<Particle> new_particles;

  std::uniform_int_distribution<int> particle_chooser(0, num_particles - 1);
  // choose random particle to start at
  auto index = particle_chooser(gen);
  auto beta = 0.0;
  auto wmax = std::max_element(particles.begin(), particles.end(),
                               [](const Particle &a, const Particle &b) {return a.weight < b.weight;})->weight;

  // choose beta between 0 and wmax (uniform distribution)
  std::uniform_real_distribution<double> beta_chooser(0.0, wmax);

  // for each particle, sample to ensure weighted choice
  for (int i = 0; i < num_particles; ++i) {
    beta += 2 * beta_chooser(gen);
    while (particles[index].weight < beta) {
      beta -= particles[index].weight;
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }

  particles = new_particles;
}

std::string ParticleFilter::getAssociations(Particle best) {
  std::vector<int> v = best.associations;
  std::stringstream ss;
  std::copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
std::string ParticleFilter::getSenseX(Particle best) {
  std::vector<double> v = best.sense_x;
  std::stringstream ss;
  std::copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
std::string ParticleFilter::getSenseY(Particle best) {
  std::vector<double> v = best.sense_y;
  std::stringstream ss;
  std::copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

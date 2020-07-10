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
using Eigen::MatrixXd;
using Eigen::VectorXd;

void ParticleFilter::init(double x, double y, double theta, double std[], int num_particles_arg)  {
  /**
   * Setting the number of particles. Initializing all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Adding random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  num_particles = num_particles_arg;

  std::default_random_engine generator;
  std::normal_distribution<double> x_distribution(x,std[0]);
  std::normal_distribution<double> y_distribution(y,std[1]);
  std::normal_distribution<double> theta_distribution(theta,std[2]);

  for (int i=0; i<num_particles; i++){
      Particle particle;
      particle.id = i;
      particle.x = x_distribution(generator);
      particle.y = y_distribution(generator);
      particle.theta = theta_distribution(generator);
      particle.weight = 1.0;
      particles.push_back(particle);
  }

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Adding measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
   std::default_random_engine generator;
   std::normal_distribution<double> x_noise(0,std_pos[0]);
   std::normal_distribution<double> y_noise(0,std_pos[1]);
   std::normal_distribution<double> theta_noise(0,std_pos[2]);

   //std::cout<< "Predicting!" << std::endl;
   for(auto &particle: particles){
       if(fabs(yaw_rate)>0.000001){
           particle.x = particle.x + velocity/yaw_rate*(sin(particle.theta + yaw_rate*delta_t)-sin(particle.theta)) + x_noise(generator);
           particle.y = particle.y + velocity/yaw_rate*(cos(particle.theta) - cos(particle.theta + yaw_rate*delta_t)) + y_noise(generator);
           particle.theta = particle.theta + yaw_rate * delta_t + theta_noise(generator);}
       else{
           particle.x = particle.x + velocity * delta_t * cos(particle.theta) + x_noise(generator);
           particle.y = particle.y + velocity * delta_t * sin(particle.theta) + y_noise(generator);
           particle.theta = particle.theta + theta_noise(generator);
       }
   }

    //std::cout<< "Finished predicting!" << std::endl;

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

    double sx = std_landmark[0];
    double sy = std_landmark[1];
    double sx2 = sx*sx;
    double sy2 = sy*sy;
    double norm = 1.0/(M_2_PI*sx*sy);
    double sr2 = sensor_range*sensor_range;

    for (auto &particle: particles){
        particle.weight = 1.0;
        vector<int> associations;
        vector<double> sense_x;
        vector<double> sense_y;

        MatrixXd H = MatrixXd(3,3);
        //matrix for transforming observations to map coordinates
        H << cos(particle.theta), -sin(particle.theta),  particle.x,
        sin(particle.theta), cos(particle.theta),   particle.y,
        0,                   0,                     1;

        //Create a vector of observations in map coordinates
        vector<LandmarkObs> observations_m;
        for(auto observation: observations){
            VectorXd obs_c = VectorXd(3);
            obs_c << observation.x, observation.y, 1.0;
            VectorXd obs_m = H*obs_c;
            LandmarkObs observation_m;
            observation_m.x = obs_m[0];
            observation_m.y = obs_m[1];
            observation_m.id = observation.id;
            observations_m.push_back(observation_m);
        }

        //Find predicted landmarks (landmarks closer than sensor_range)
        vector<LandmarkObs> predicted;
        for (auto landmark: map_landmarks.landmark_list) {
            double delta_x = particle.x - landmark.x_f;
            double delta_y = particle.y - landmark.y_f;
            if(sqrt(delta_x*delta_x + delta_y*delta_y) < sensor_range){
                LandmarkObs predicted_obs;
                predicted_obs.x = landmark.x_f;
                predicted_obs.y = landmark.y_f;
                predicted_obs.id = landmark.id_i;
                predicted.push_back(predicted_obs);
            }
        }



        //find the closest predicted landmark for each observation
        for (auto observation: observations_m){
            double min_distance = sensor_range;
            vector<LandmarkObs, std::allocator<LandmarkObs>>::iterator associated_landmark;
            for (auto it = predicted.begin(); it != predicted.end(); it++){
                double delta_x = it->x - observation.x;
                double delta_y = it->y - observation.y;
                double distance = sqrt(delta_x*delta_x + delta_y*delta_y);
                if (distance<min_distance){
                    min_distance = distance;
                    associated_landmark = it;
                }
            }
            if(min_distance == sensor_range){
                double pdf = norm * exp(-0.5*((sr2)/(sx2) + (sr2)/(sy2)));
                particle.weight = particle.weight * pdf;
            }
            else{
                double delta_x = associated_landmark->x - observation.x;
                double dx2 = delta_x*delta_x;
                double delta_y = associated_landmark->y - observation.y;
                double dy2 = delta_y*delta_y;
                //std::cout<< "Delta x: " << delta_x << std::endl;
                //std::cout<< "Delta y: " << delta_y << std::endl;

                double pdf = norm * exp(-0.5*(dx2/sx2 + dy2/sy2));
                //std::cout<< "Pdf: " << pdf << std::endl;
                particle.weight = particle.weight * pdf;

                associations.push_back(associated_landmark->id);
                sense_x.push_back(observation.x);
                sense_y.push_back(observation.y);

                /**
                 * TODO: remove the predicted associated landmark to avoid comparing
                 *   already associated landmarks with remaining observations  */
                //predicted.erase(associated_landmark);
            }
        }
        SetAssociations(particle, associations, sense_x, sense_y);
        //std::cout<< "Particle Weight: " << particle.weight << std::endl;
    }
}

void ParticleFilter::resample() {
  /**
   * Resamples particles with replacement with probability proportional
   * to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   std::default_random_engine generator;
   std::uniform_real_distribution<double> uniform_distribution = std::uniform_real_distribution<double>(0.0,1.0);
   vector<Particle> sampled_particles;

   double max_weight = 0.0;
   for (auto particle: particles) {
       if(particle.weight>max_weight){
           max_weight=particle.weight;
       }
   }
   //std::cout<< "Max Weight: " << max_weight << std::endl;

   int particle_index = int(uniform_distribution(generator)*num_particles);
   double beta = 0.0;
   double max_weight_since_last_sample = 0.0;
   //std::cout<< "Resampling..." << std::endl;
   while(sampled_particles.size()<num_particles){
       //std::cout<< "Particles sampled: " << sampled_particles.size() << std::endl;
       Particle particle = particles[particle_index];
       if (particle.weight>max_weight_since_last_sample){max_weight_since_last_sample=particle.weight;}
       //std::cout<< "Particle weight: " << particle.weight << " Beta: "<< beta << " Max Particle Weight Since Last Sample: " << max_weight_since_last_sample << std::endl;
       if(beta>particle.weight){
           beta = beta - particle.weight;
           particle_index = particle_index + 1;
           if (particle_index>num_particles){particle_index = 0;}
       }
       else{
           sampled_particles.push_back(particle);
           beta = beta + uniform_distribution(generator)*2.0*max_weight;
           max_weight_since_last_sample = 0.0;
       }
   }
   particles = sampled_particles;
   //std::cout<< "Finished resampling!" << std::endl;
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
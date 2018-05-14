/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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
#define EPS 0.001
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	if(is_initialized)
		return;

	random_device rd;
	default_random_engine generator( rd() );
	gen = generator; 
	num_particles = 100;
	is_initialized = true;
	double std_x, std_y, std_theta;
	std_x = std[0];
	std_y = std[1];
	std_theta = std[3];
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for(int i=0; i < num_particles; i++){
		Particle pt;
		pt.id = i;
		pt.x = dist_x(gen);
		pt.y = dist_y(gen);
		pt.theta = dist_theta(gen);
		pt.weight = 1.0;
		particles.push_back(pt);
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	double std_x, std_y, std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

	double v_;
	for(int i=0; i < num_particles; i++){
		if(fabs(yaw_rate) < EPS) {
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		} else {
			v_ =  velocity/yaw_rate;
			particles[i].x += v_ * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += v_ * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t) );
			particles[i].theta += yaw_rate * delta_t;
		}
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta +=  dist_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	int nearest;
	double dist_m, dist_;

	for(int i=0; i < observations.size(); i++){
		nearest = -1;
		dist_m = numeric_limits<double>::max();;
		dist_ = -1;

		for(int j=0; j < predicted.size(); j++){
			dist_ = dist(observations[i].x,  observations[i].y, predicted[j].x, predicted[j].y);
			if(dist_ < dist_m){
				dist_m = dist_;
				nearest = predicted[j].id ;
			}
		}

		observations[i].id = nearest;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	vector<LandmarkObs> marks;
	double std_x = std_landmark[0];
	double std_y = std_landmark[1];
	double dist_;

	for(int i=0; i < num_particles; i++){
		for(int j=0; j < map_landmarks.landmark_list.size(); j++){
			dist_ = dist(particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);

			if( dist_ <= sensor_range ) {
				LandmarkObs l_;
				l_.id = map_landmarks.landmark_list[j].id_i;
				l_.x = map_landmarks.landmark_list[j].x_f;
				l_.y = map_landmarks.landmark_list[j].y_f;
				marks.push_back(l_);
			}
		}

		vector <LandmarkObs> obs;
		obs = transformCoordinates(observations, particles[i].theta, particles[i].x, particles[i].y);
		dataAssociation(marks, obs);
		updateParticleWeight(particles[i], obs, marks, std_x, std_y);

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	vector<double> weights;
	double mw = numeric_limits<double>::min();

	for(int i =0; i < num_particles; i++){
		weights.push_back(particles[i].weight);
		if(particles[i].weight > mw)
			mw = particles[i].weight;
	}

	random_device seed;
	mt19937 random_generator(seed());
	// sample particles based on their weight
	discrete_distribution<> sample(weights.begin(), weights.end());
	vector<Particle> new_particles(num_particles);
	for(auto & p : new_particles)
		p = particles[sample(random_generator)];
	
	particles = move(new_particles);
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

double ParticleFilter::gaussMulti(double x_diff, double y_diff, double varx, double vary){
	double a = 1/(2 * M_PI * varx * vary);
	double b = (x_diff * x_diff) / (2 * varx * varx) + (y_diff * y_diff) / (2 * vary * vary);
	return a * exp(-b);
}

void ParticleFilter::updateParticleWeight(Particle& particle, std::vector<LandmarkObs> obs, std::vector<LandmarkObs> marks, double std_x, double std_y){
	particle.weight = 1.0;
	int mark_index;
	double weight, diff_x, diff_y;

	for(int i=0; i < obs.size(); i++) {
		mark_index = -1;
		for(int j=0; j < marks.size(); j++){
			if(marks[j].id == obs[i].id){
				mark_index = j;
			}
		}

		weight = 0.0;
		if(mark_index != -1){
			diff_x = obs[i].x - marks[mark_index].x;
			diff_y = obs[i].y - marks[mark_index].y;
			weight = gaussMulti(diff_x, diff_y, std_x, std_y);
		}

		if(weight == 0.0) {
			particle.weight = particle.weight * EPS;
		} else {
			particle.weight = particle.weight *  weight;
		}

	}
}

vector <LandmarkObs> ParticleFilter::transformCoordinates(vector <LandmarkObs>  observations, double theta, double x, double y){
	vector <LandmarkObs> obs;
	for(int i=0; i < observations.size(); i++){
		LandmarkObs l_;
		l_.id = observations[i].id;
		l_.x = cos(theta)*observations[i].x - sin(theta)*observations[i].y + x;
		l_.y = sin(theta)*observations[i].x + cos(theta)*observations[i].y + y;
		obs.push_back(l_);
	}
	return obs;
}
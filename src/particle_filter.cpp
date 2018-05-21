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

#define NUM_PARTICLES 50

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> ang_theta(theta, std[2]);

	num_particles = NUM_PARTICLES;

	for(int i = 0; i < num_particles; i++)
	{
		Particle P;
		P.id	 = i;
		P.x 	 = dist_x(gen);
		P.y 	 = dist_y(gen);
		P.theta  = ang_theta(gen);
		P.weight = 1;
		particles.push_back(P);
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::default_random_engine gen;

	for(int i = 0; i < particles.size(); i++)
	{
		double temp_x, temp_y, temp_theta;

		if(fabs(yaw_rate) < 0.0001)
		{
			temp_x 		= particles[i].x + velocity*delta_t*cos(particles[i].theta);
			temp_y 		= particles[i].y + velocity*delta_t*sin(particles[i].theta);
			temp_theta 	= particles[i].theta;
		}
		else
		{
			temp_x 		= particles[i].x + (velocity / yaw_rate)*(sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			temp_y 		= particles[i].y + (velocity / yaw_rate)*(-cos(particles[i].theta + yaw_rate * delta_t) + cos(particles[i].theta));
			temp_theta 	= particles[i].theta + (yaw_rate*delta_t);
		}

		std::normal_distribution<double> dist_x(temp_x, std_pos[0]);
		std::normal_distribution<double> dist_y(temp_y, std_pos[1]);
		std::normal_distribution<double> ang_theta(temp_theta, std_pos[2]);

		particles[i].x 		= dist_x(gen);
		particles[i].y 		= dist_y(gen);
		particles[i].theta 	= ang_theta(gen);

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<int> & associations, std::vector<double> sense_x, std::vector<double> sense_y) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for(int i = 0; i < associations.size(); i++)
	{
		double min_dist = 1.0E99;
		for(int j = 0; j < predicted.size(); j++)
		{
			double dist = sqrt(pow((sense_x[i] - predicted[j].x),2) + pow((sense_y[i] - predicted[j].y),2));
			if(dist < min_dist)
			{
				associations[i] = j;
				min_dist = dist;
			}
		}
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
	std::vector<LandmarkObs> predicted;

	for(int i = 0; i < num_particles; i++)
	{
		std::vector<int> associations;
		vector<double> sense_x;
		vector<double> sense_y;
		double temp_x, temp_y;

		double p_prop;
		double p_x 			 = particles[i].x;
		double p_y 			 = particles[i].y;
		double p_theta 		 = particles[i].theta;
		int idx = 0;

		for(int j = 0; j < map_landmarks.landmark_list.size();j++)
		{
			LandmarkObs temp {idx,
							  map_landmarks.landmark_list[j].x_f,
						      map_landmarks.landmark_list[j].y_f};

			double dist = sqrt(pow((temp.x - p_x),2) + pow((temp.y - p_y),2));

			if(dist <= sensor_range)
			{
				predicted.push_back(temp);
				idx++;
			}
		}

		for(int j = 0; j < observations.size(); j++)
		{
			temp_x = p_x + (observations[j].x * cos(p_theta)) - (observations[j].y * sin(p_theta));
			temp_y = p_y + (observations[j].x * sin(p_theta)) + (observations[j].y * cos(p_theta));

			sense_x.push_back(temp_x);
			sense_y.push_back(temp_y);
			associations.push_back(-1);
		}

		dataAssociation(predicted, associations, sense_x, sense_y);
		//cout << "Set Association" << endl;
		//SetAssociations(particles[i], associations, sense_x, sense_y);

		particles[i].weight = 1.0;

		for(int j = 0; j < observations.size(); j++)
		{
			double land_mark_x  = predicted[associations[j]].x;
			double land_mark_y  = predicted[associations[j]].y;
			double obs_x 		= sense_x[j];
			double obs_y		= sense_y[j];
			double std_x		= std_landmark[0];
			double std_y		= std_landmark[1];
			double var_x		= (std_x * std_x);
			double var_y 		= (std_y * std_y);
			double PI			= 3.14159265359;
			double dist;

			p_prop = ((1/(2*PI*std_x*std_y)) * exp(-((pow((obs_x - land_mark_x) , 2)/(2*var_x)) + (pow((obs_y - land_mark_y) , 2)/(2*var_y)))));

			if(p_prop < 1.0E-100)
			{
				p_prop = 1.0E-100;
			}
			particles[i].weight *= p_prop;
		}
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::vector<Particle> p_temp;
	std::uniform_int_distribution<int> dist_index(0,num_particles);
	std::default_random_engine gen;

	int index = dist_index(gen);
	double beta = 0;
	double w_max = 0;

	// Search for maximum weight
	for(auto p : particles)
	{
		if(p.weight > w_max)
		{
			w_max = p.weight;
		}
	}

	std::uniform_real_distribution<double> dist_w(0,2*w_max);

	for(int i = 0; i < particles.size(); i++)
	{
		beta += dist_w(gen);

		while(beta > particles[index].weight)
		{
			beta = beta - particles[index].weight;
			index = (index + 1) % num_particles;
		}

		p_temp.push_back(particles[index]);


	}

	particles = p_temp;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    //particle.weight = 1;

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

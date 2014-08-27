/*
 * PI2Wholesteps.cpp
 *
 *  Created on: 11 Jun 2014
 *      Author: Sromona Chatterjee
 *      Email: chatterji.sromona.86@gmail.com
 */

#include <iostream>
#include <stdlib.h>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/Eigen"
#include <algorithm>
#include <stack>
#include <map>
#include <utility>
#include <ctime>

using namespace Eigen;
using namespace std;

#include "PI2Wholesteps.h"
#include "GenerateNoise.h"
#include "KinemodelController.cpp"

#define SRAND()     srand(rand()%RAND_MAX*32768+rand()*rand()+500+(unsigned)time(0))

extern float dt;

struct updateInfo {
    float u_new[4]; // set parameters size = no. of learned parameters here
    double new_cost;

};

//double uniform()
//{
//	//SRAND();
//	return (double)(rand()+32768*rand())/1073741824;
//}
//double gaussian()
//{
//	return sqrt(-2.0*log(uniform()))*cos(2*M_PI*uniform());
//}

double uniform() {
  return double(rand()) / RAND_MAX;
  //return (double)(rand()+(RAND_MAX+1)*rand())/((RAND_MAX+1)*(RAND_MAX+1));
}
double gaussian() {
  return sqrt(-2.0 * log(uniform())) * cos(2 * M_PI * uniform());
}

Eigen::MatrixXd rolloutCost(vector<double> noise1, vector<double> noise2,
    vector<double> noise3, vector<double> noise4, float y[], float t[],
    float u[], int rollout, float goal[]) {
  kinecontroller k;
  costfunction costf;
  MatrixXd noise_matrix(rollout, 5); //4 noises for 4 parameters getting learned and 1 cost for each rollout

  // running robot using controller to get reached position
  for (int i = 0; i < rollout; i++) {
    float u_noise[4];
    // generating noisy parameters
    //noise_matrix(i,0)=0.3*noise1[i];
    //noise_matrix(i,1)=0.3*noise2[i];
    //noise_matrix(i,2)=0.3*noise3[i];
    //noise_matrix(i,3)=0.3*noise4[i];

    noise_matrix(i, 0) = gaussian();
    noise_matrix(i, 1) = gaussian();
    noise_matrix(i, 2) = gaussian();
    noise_matrix(i, 3) = gaussian();

    u_noise[0] = u[0] + noise_matrix(i, 0);
    u_noise[1] = u[1] + noise_matrix(i, 1);
    u_noise[2] = u[2] + noise_matrix(i, 2);
    u_noise[3] = u[3] + noise_matrix(i, 3);

    //cout<<"\n new u with noise is: \n"<<u_noise[0];
    //calling the controller with noisy parameters
    MatrixXd dy1;
    float dy2[6] = { 0, 0, 0, 0, 0, 0 };
    float y_new[6] = { y[0], y[1], y[2], y[3], y[4], y[5] };
    //for(int j=0;j<sizeof(t);j++)
    for (int j = 0; j < 4; j++) {

      MatrixXd dy = k.kinemodel(y_new, u_noise);
      // taking the timesteps t and change dy
      dy1 = dy * dt;
      dy2[0] = dy1(0, 0);
      dy2[1] = dy1(1, 0);
      dy2[2] = dy1(2, 0);
      //cout<<"\n dy2 is: \n"<<dy2[0]<<" "<<dy2[1]<<" "<<dy2[2];
      // getting new y state vector
      for (int k = 0; k < 6; k++) {
        y_new[k] = y_new[k] + dy2[k];
      }
    }

    //using cost function to get noisy final cost
    double rollout_cost = costf.getcost(goal, y_new);
    //cout<<"\n new cost is: \n"<<rollout_cost;

    // saving noises and cost for use in PI2 update rule
    noise_matrix(i, 4) = rollout_cost;

  }

  return noise_matrix;
}

updateInfo updateRule(Eigen::MatrixXd noiseMatrix, float y[], float t[],
    float u[], int rollout, float goal[]) {
  kinecontroller k1;
  costfunction costf1;
  //int param_size=sizeof(u);
  int param_size = 4;
  float u_update[param_size];
  double update_cost;
  updateInfo info;

  u_update[0] = u[0];
  u_update[1] = u[1];
  u_update[2] = u[2];
  u_update[3] = u[3];

  //cout<<"\n in update rule, noise matrix is: \n"<<noiseMatrix;
  //cout<<"\n in update rule, updated u is: \n"<<u_update[0]<<" "<<u_update[1]<<" "<<u_update[2]<<" "<<u_update[3];

  /********** PI2 update rule starts here***********************************/

  MatrixXd expCost(rollout, 1); // of noisy costs
  MatrixXd immecost(rollout, 1);
  for (int i = 0; i < rollout; i++) {
    immecost(i, 0) = noiseMatrix(i, 4);

  }
  for (int i = 0; i < rollout; i++) {
    double a = -30
        * ((immecost(i, 0) - immecost.minCoeff())
            / (immecost.maxCoeff() - immecost.minCoeff()));
    expCost(i, 0) = exp(a);

  }

  double sumExp = expCost.sum();

  MatrixXd pWeighting(rollout, 1);
  for (int i = 0; i < rollout; i++) {
    pWeighting(i, 0) = expCost(i, 0) / sumExp;

  }
  //cout<<"\n weighting \n"<<pWeighting;
  /* correction to parameters here*/
  for (int i = 0; i < param_size; i++) {
    MatrixXd correctionMatrix(rollout, 1);
    for (int j = 0; j < rollout; j++) {
      correctionMatrix(j, 0) = pWeighting(j, 0) * noiseMatrix(j, i);
      //cout<<"\n noise \n"<<noiseMatrix(j,i);

    }
    double correction1 = correctionMatrix.sum();
    u_update[i] = u_update[i] + correction1; // final parameter update

    /*if (u_update[i]>1)
     u_update[i]=1;
     if (u_update[i]<-1)
     u_update[i]=-1;*/
    info.u_new[i] = u_update[i];

  }

  /********** PI2 update rule ends here******************************************/

  // running robot using controller to get reached position
  //calling the controller with updated parameters
  MatrixXd dy11;
  float dy21[6] = { 0, 0, 0, 0, 0, 0 };
  float y_update[6] = { y[0], y[1], y[2], y[3], y[4], y[5] };
  //cout<<"time "<<sizeof(t)<<" \n";
  //for(int j=0;j<sizeof(t);j++)
  for (int j = 0; j < 4; j++) {
    MatrixXd dy = k1.kinemodel(y_update, u_update);
    // taking the timesteps t and change dy
    dy11 = dy * dt;
    dy21[0] = dy11(0, 0);
    dy21[1] = dy11(1, 0);
    dy21[2] = dy11(2, 0);
    //cout<<"\n dy21 is: \n"<<dy21[0]<<" "<<dy21[1]<<" "<<dy21[2];
    // getting new y state vector
    for (int k = 0; k < 6; k++) {
      y_update[k] = y_update[k] + dy21[k];
    }
  }

  //using cost function to get noisy final cost
  update_cost = costf1.getcost(goal, y_update);

  // returning updated parameter set and noise-free cost
  info.new_cost = update_cost;
  return info;
}

Eigen::MatrixXd PI2Wholesteps::wholesteps(float y[], float t[], float phi[],
    float goal[]) {
  float u[4] = { 0, 0, 0, 0 };  // initialize u i.e. parameters getting learned
  int rollout = 40;                      // set number of rollouts; even number
  int update_no = 80;                        // set number of updates
  vector<double> update_cost;              // vector with noise-free cost
  // after every update
  float u_updated[4];
  updateInfo b;
  b.u_new[0] = u[0];
  b.u_new[1] = u[1];
  b.u_new[2] = u[2];
  b.u_new[3] = u[3];

  // getting noises
  //StatisticalDistribution snd;
  //vector<double> noiseALL = snd.standardNoise(rollout*update_no*4);

  //call PI2 update
  for (int i = 0; i < update_no; i++) {
    u_updated[0] = b.u_new[0];
    u_updated[1] = b.u_new[1];
    u_updated[2] = b.u_new[2];
    u_updated[3] = b.u_new[3];

    //call PI2 rollouts
    MatrixXd noise_matrix1;

    vector<double> noise1;
    vector<double> noise2;
    vector<double> noise3;
    vector<double> noise4;

    /*for (int i=0;i<rollout;i++)
     {
     double a=noiseALL.back();
     noise1.push_back(a);
     noiseALL.pop_back();

     a=noiseALL.back();
     noise2.push_back(a);
     noiseALL.pop_back();

     a=noiseALL.back();
     noise3.push_back(a);
     noiseALL.pop_back();

     a=noiseALL.back();
     noise4.push_back(a);
     noiseALL.pop_back();
     }*/

    noise_matrix1 = rolloutCost(noise1, noise2, noise3, noise4, y, t,
        u_updated, rollout, goal);
    //cout<<"\n Noise Matrix is: \n"<<noise_matrix1;

    b = updateRule(noise_matrix1, y, t, u_updated, rollout, goal);
    update_cost.push_back(b.new_cost);
    //cout<<"\n cost is: \n"<<b.new_cost;

  }

  //print noise-free costs to see convergence
  if (verbose)
  {
    for (int i = 0; i < update_no; i++) {
      cout << "\n noise free cost at the end of update no. " << i + 1
          << " is : \n" << update_cost[i];
    }
  }
  //cout<<"\n final u values in PI2 are: \n"<<b.u_new[0]<<","<<b.u_new[1];
  //cout<<"\n remaining u values "<<b.u_new[2]<<","<<b.u_new[3];

  Eigen::MatrixXd update_u_matrix(1, 4);
  update_u_matrix(0, 0) = b.u_new[0];
  update_u_matrix(0, 1) = b.u_new[1];
  update_u_matrix(0, 2) = b.u_new[2];
  update_u_matrix(0, 3) = b.u_new[3];

  return update_u_matrix;
}


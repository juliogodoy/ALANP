#ifndef __SIMULATIONDEFS_H_INCLUDED__   // if x.h hasn't been included yet...
#define __SIMULATIONDEFS_H_INCLUDED__ 

#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"
#include <stdlib.h>
#include <cstdlib>
#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

//**************Default Simulation Parameters*********************//

float updateProbability=0.25; // In average, each agent makes decision every (1/updateProbability) timesteps (directly dep in timestep length) 
float simTimeStep=0.05; // Length of the timestep, in each second there are (1/simTimeStep) timesteps
int sidesize=25; //Size of each side of the environment of the Crowd scenario
bool randomPert=1; //Enables small random perturbations on the preferred velocities computed
bool finalize=0; //  marks the end of the simulation

const int numNeighbors=10;//number of neighbors that each agent considers
const float neighborDistance=15.0f; //max distance that agents can perceive neighbors
const float timeHorizonORCA=5.0f;  // time horizon to determine collisions with other agents
const float timeHorizonObstORCA=1.3f;// time horizon to determine collisions with obstacles
const float radiusORCA=0.5f;  // distance that the agents want to keep from other agents
const float maxSpeedORCA=1.5f; //maximum speed that agents can move with

int greedyorUCB=1; //1= ALAN
float epsilondec=10;
float SimScore=0;
float baseScore, cumAccel=0;
int agentViewed;
int colisiones=0; 
int decisionSteps=0, sample_time_spent=0, action_change=0, action_step=0;
float globalAvgReward=0, globalGoalOriented=0;
double total_time_spent=0;

float coord_factor; // Coordination factor, that balances between the goal progress and the politeness of the agents
static const int timeHorizon=1; //Number of (future) timesteps to simulate to evaluate the effect of each action
int threshold=80000; //Maximum number of timesteps before the simulation is declared not finished.3
int sizeTW;
int currentVel[750];
int TimeWindow[750][10000];
bool winning[750];
float TWscore[750][10000], maxAccel=-10;
float Selfish[750][750], Polite[750][750],LastActionEstimate[750][750];
float totalGoalOriented[750];
float UCBound[750][750];
float  realProg[750], epsilon[750], cumReward[750];
int Chosen[750][750];
int count[750][750];

float lastReward[750];
int lastAction[750];
bool changeAction[750];

float cumavg[15];
float temp[750];
float Boltz[750][750];
float UCBconst[750];
float ori[750];
float defaultValue[750][750];

    float AvgTime=0;
    int totalcounts=0;
    int totnumcan=0;
    int samp=0;


//General variable declaration
RVO::RVOSimulator* sim;
int algo,Agents, scenario,Actions,chosenAction[750], iteration, timestep, lastFrameTime = 0, totalnotingoal,totalingoal,finalIteration;
bool isinGoal[750],visualizer;
float Qvalues[750][50];
RVO::Vector2 goalvectors[750],goalVector,prevVel[750],postVel[750], initPos[750];
std::vector<RVO::Vector2> goals;
float angle,dist,actionVector[50],actionVectorMag[50],finalTime=-100,RewardAvg[750][50],TimetoGoal[750],goaldistance[750],ActionSpeed[750][50],ActionDir[750][50], goalx[750], goaly[750],totalrewardsimulation=0, totalrewarditeration, needtoUpdate[750];


#endif

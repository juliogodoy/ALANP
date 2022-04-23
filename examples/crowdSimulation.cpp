/*Code used for the C-Nav vs ORCA experiments
 * Based on the RVO2 library.
 * Author: Julio Godoy Del Campo
 * Please send all comments and/or questions to juliogodoy@gmail.com
 *
 * */
#include <iostream>
#include <vector>
#include <fstream>
//#include <gl/glut.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <time.h>
#include <string.h>
#include "simulationDefs.h"



#if HAVE_OPENMP || _OPENMP
#include <omp.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#ifdef __APPLE__
#include <RVO/RVO.h>
#else
#include "RVO.h"
#endif

#ifndef M_PI
static const float M_PI = 3.14159265358979323846f;
#endif

RVO::Vector2 firstGoal = RVO::Vector2(-30, 0.0);

void InitGL(void)     // OpenGL function
{
    
    glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
    glClearColor(0.5f, 0.5f, 0.5f, 0.5f);				// Black Background
    glClearDepth(1.0f);									// Depth Buffer Setup
    glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
    glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
    glEnable ( GL_COLOR_MATERIAL );
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}


void reshape(int width, int height) //OpenGL function
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    
    
    float q = width/(float)height;
    
    switch(scenario)
    {
            
        case 1: glOrtho(-10,sidesize+10,-10,sidesize+10, -2, 20);
            break;
            
        case 2: gluOrtho2D(-q*45, q*45, -45, 45 );
            break;
            
        case 3: gluOrtho2D(-q*20, q*20, 10, 50 );
            break;
            
        case 4: gluOrtho2D(-q*25, q*25, -25, 25 );
            break;
            
        case 5: glOrtho(-20,10,-10,10, -2, 20);
            break;
            
        case 6: gluOrtho2D(-q*15, q*15, -15, 15 );
            break;
            
        case 7: glOrtho(-30,30,-20,20, -2, 20);
            break;
        case 8: glOrtho(-10,sidesize+10,-10,sidesize+10, -2, 20);
            break; 
           
            
            //gluOrtho2D(-2,2,-2,2);
            
            
            
    }
    
    
    
    glMatrixMode(GL_MODELVIEW);
}

void countCollisions()
{
	int i,j;
	
	for(i=0;i<Agents;i++)
	{
		
		
			if(isinGoal[i])
			{ 
				continue;
				//      std::cout << " Agent: " << i <<" Alive\n";
			}
			
			for(j=0;j<Agents;j++)
			{
				
				if(isinGoal[j])
				{
					continue;
					//      std::cout << " Agent: " << i <<" Alive\n";
				}
				
				if(i!=j)
				{
					//RVO::Vector2 = agentdistance;
					float distancia;
					distancia=RVO::abs(sim->getAgentPosition(i)-sim->getAgentPosition(j));
					
					//std::cout<< i << "  ALMOST collision with " << j << " distnace: " << distancia<< "\n";
					if((i==15)&&(distancia<1.5))
					{
					//	std::cout<< "ALMOST collision with " << " distnace: " << distancia<< "\n";
					//	char t=getchar();
				      }
					
					if(distancia<.95*(sim->getAgentRadius(i)+ sim->getAgentRadius(j) ) )
					{
						//	std::cout << colisiones;
						colisiones++;
						
						//std::cout<< "Collision with " << " real pos: " << sim->getAgentPosition(j)<< "\n";
						//	char o= getchar();
					}
					
					
				}
				
			}
		
	}
	
}

void calcAccel()
{
	
	int i,j;
	
	for(i=0;i<Agents;i++)
	{
		
		
			if(isinGoal[i])
			{ 
				continue;
				//      std::cout << " Agent: " << i <<" Alive\n";
			}
			
			cumAccel=cumAccel+RVO::abs(postVel[i]-prevVel[i]);
			
			
			
			if(maxAccel<RVO::abs(postVel[i]-prevVel[i]))
			{
				
				
				RVO::Vector2 theoryAcc= postVel[i]-prevVel[i];
				RVO::Vector2 normalizedAcc=normalize(theoryAcc);
				RVO::Vector2 newVel= prevVel[i]+normalizedAcc*maxAccel;
				
				
			//maxAccel= 	RVO::abs(postVel[i]-prevVel[i]);
			}
			
		}
	
}

void idle(void) //OpenGL function
{
    glutPostRedisplay();
}

RVO::Vector2 chooseGoal(int i)
{
    if(isinGoal[i]==0)
    {
        
        if(scenario==5) //Could be Hallway or Congested Exit
        {
            RVO::Vector2 finalGoal = RVO::Vector2(-30, 0.0);
            RVO::Vector2 goalPos = firstGoal;
            
            
            if (sim->queryVisibility(sim->getAgentPosition(i), finalGoal, sim->getAgentRadius(i)))
            {
                
                
                goals[i] = finalGoal;
                
                
            }
            else
            {
                goals[i] = firstGoal;
            }
            
        }
        
        return goals[i];
    }
}

void addTimeWindow2(int i, int chosenAct,float newReward)
{
    int j;
    
   
    
    for(j=(sizeTW-1);j>0;j--)
    {
        TimeWindow[i][j]=TimeWindow[i][j-1];
        TWscore[i][j]=TWscore[i][j-1];
        Chosen[i][TimeWindow[i][j]]=Chosen[i][TimeWindow[i][j]]+1;
    }
    
   
    
    TimeWindow[i][0]=chosenAct;
    Chosen[i][chosenAct]=Chosen[i][chosenAct]+1;
    TWscore[i][0]=newReward;
    
   
    
    
    //Display the resulting timewindow
    if(i==agentViewed)
    {
        for(j=0;j<sizeTW;j++)
        {
        //    std::cout << " Timewindow " << j << ": " << TimeWindow[i][j] <<" | " << TWscore[i][j]<< " \n";
        }
        
    }
    
    
}


int Boltzmann(int i)
{  
   int candidateAction[Actions];
   int numCandidates=0;
	int j;
	
	for (j=0; j<Actions; j++) 
	{
		Qvalues[i][j]= LastActionEstimate[i][j];
		candidateAction[j]=0;
    }
	for (j=0; j<Actions; j++) 
	{
		
		if(i==agentViewed)
			{
				
			std::cout << "Agent " << i << " comparing CANDIDATE Action: " << j << " with defVal: " << defaultValue[i][j] <<" with actual " << Qvalues[i][currentVel[i]] << "\n"; 	
			}
		
		if((defaultValue[i][j]>  Qvalues[i][currentVel[i]])||(sim->getAgentNumAgentNeighbors(i)==0))//realProg[i])
		{
		    candidateAction[j]=1;	
			numCandidates=numCandidates+1;
			
			if(i==agentViewed)
			{
				
			std::cout << " Action CANDIDATE: " << j << " added with defVal: " << defaultValue[i][j] << "\n"; 	
			}
		}
		
	}
	
	if(numCandidates==0)
	{
	 	
		totnumcan=  totnumcan+1;
	}
	else
	{
		totnumcan=  totnumcan+numCandidates;
	}
	
	
   
   //std::cout << " AGENT " << i <<" NUM CANDIDATES : " <<numCandidates << "\n"; 
   samp=samp+1;
	
	
	float sumBoltz=0;
	for (j=0; j<Actions; j++) 
	{   // Qvalues[i][j]= LastActionEstimate[i][j];
		
		if(candidateAction[j]>0)
		{
		sumBoltz=sumBoltz+exp(Qvalues[i][j]/temp[i]);
		//sumBoltz=sumBoltz+((Qvalues[i][j])/(1));
		
	}
		
	}
	
	for (j=0; j<Actions; j++) 
	{
		
		if(candidateAction[j]>0)
		{
		Boltz[i][j]=100*(exp(Qvalues[i][j]/temp[i]))/sumBoltz;
		//Boltz[i][j]=100*(((Qvalues[i][j])/(1)))/sumBoltz;
	}
	else
	{
		Boltz[i][j]=0;
		}
		if((!isinGoal[i])&&(i==agentViewed)&&(candidateAction[j]>0))//(i==agentViewed)
                    {
                       std::cout << "Action "<< j << " Boltzmann value of : " << Boltz[i][j] << " with Qval: " << Qvalues[i][j] << "\n";
                       
                    }
		
		
	}
	
	float probability= rand()%100;
		if(i==agentViewed)//(i==agentViewed)
                    {
                       std::cout << "Boltzmann PROB of " <<j << " : " << probability << "\n";
                       
                    }
	float current=0;
	
	for (j=0; j<Actions; j++) 
	{
		current=current+Boltz[i][j];
		
		if(probability<current)
		{
			
			if(j!=chosenAction[i])
			{
				
				lastReward[i]= realProg[i];
				lastAction[i]=chosenAction[i];
				changeAction[i]=1;
				if(i==agentViewed)//(i==agentViewed)
				{
				   std::cout << "Agent " << i << " going to chage action from  " << chosenAction[i] << " to " << j << " and last reward is " << 	lastReward[i] <<"\n";
				
				}
				action_change=action_change+1;
				
				}
				action_step=action_step+1;
			
			return j;
		}
		
	}
	return 0;
	
	
}

int BoltzmannOld(int i)
{   
   	int j;
	float sumBoltz=0;
	for (j=0; j<Actions; j++) 
	{   // Qvalues[i][j]= LastActionEstimate[i][j];
		
		
		sumBoltz=sumBoltz+exp(Qvalues[i][j]/temp[i]);
	
		
	}
	
	for (j=0; j<Actions; j++) 
	{
		
		
		Boltz[i][j]=100*(exp(Qvalues[i][j]/temp[i]))/sumBoltz;
	
		if(i==agentViewed)
                    {
                        std::cout << "Boltzmann value of " <<j << " : " << Boltz[i][j] << "\n";
                       
                    }
		
		
	}
	
	float probability= rand()%100;
	float current=0;
		if(i==agentViewed)
                    {
                       std::cout << "Boltzmann PROB of " <<j << " : " << probability << "\n";
                       
                    }
	
	for (j=0; j<Actions; j++) 
	{
		current=current+Boltz[i][j];
		
		if(probability<current)
		{
			return j;
		}
		
	}
	return 0;
	
	
}




bool checkOverlap(int i)  //Checks whether the position of agent i overlaps the position of an agent previously located (in which case it returns 0)
{
    for (int j=0;j<i;j++)
    {
        if(RVO::abs(sim->getAgentPosition(j)-sim->getAgentPosition(i))<2*sim->getAgentRadius(i))
        {
            return 0;
        }
        
    }
    
    //std::cout << " Agent " << i << " done!\n";
    return 1;
}

void genScenario() //Generates the positions of the agents in the Crowd scenario
{
    float xpos,xgpos, ypos,ygpos;
    srand (time(NULL));
    RVO::Vector2 tempgoal,initpos;
    std::ofstream initpositions;
    initpositions.open("positions.txt", std::fstream::trunc);
    
    //std::cout << " Start assigning agents \n";
    for (int i=0; i<Agents; i++)
    {
        
        sim->addAgent(RVO::Vector2());
        
        do{
            xpos=rand()%(sidesize+1);
            ypos=rand()%(sidesize+1);
            
            initpos=RVO::Vector2(xpos,ypos);
            sim->setAgentPosition(i,initpos);
          // std::cout <<"+";
            do{
                xgpos=rand()%(sidesize+1);
                ygpos=rand()%(sidesize+1);
                tempgoal=RVO::Vector2(xgpos,ygpos);
            }while(  (RVO::abs(tempgoal-initpos)<((float)sidesize/(float)2)));// || ((xgpos>=(((float)sidesize/(float)2)-3))&&(xgpos<=(((float)sidesize/(float)2)+3))&&(ygpos>=(((float)sidesize/(float)2)-3))&&(ygpos<=(((float)sidesize/(float)2)+3)))  );
           // std::cout <<"-";
        }while((!checkOverlap(i)));//||((xpos>=(((float)sidesize/(float)2)-3))&&(xpos<=(((float)sidesize/(float)2)+3))&&(ypos>=(((float)sidesize/(float)2)-3))&&(ypos<=(((float)sidesize/(float)2)+3))));
        
        initpositions << sim->getAgentPosition(i).x() << "\n"<< sim->getAgentPosition(i).y() << "\n" << xgpos <<"\n" << ygpos<<"\n";
        
        goals.push_back(RVO::Vector2(xgpos,ygpos));
        //std::cout << "Assigned agent " << i << ": "<< sim->getAgentPosition(i).x() << ", "<< sim->getAgentPosition(i).y() << "...Goal pos:" << xgpos <<", " << ygpos<<"\n";
        
    }
    //std::cout << " --End assigning agents \n";
    
    initpositions.close();
}

void genScenarioCong() //Generates the positions of the agents in the Crowd scenario
{
	float xpos,xgpos, ypos,ygpos;
	srand (time(NULL));
	RVO::Vector2 tempgoal,initpos;
	std::ofstream initpositions; 
	//std::cout << "Generating agents..\n";
	initpositions.open("positions.txt", std::fstream::trunc);
	for (int i=0; i<Agents; i++) 
		{
			
		sim->addAgent(RVO::Vector2());
		
		do{
			xpos= (rand()%9)-4;
			ypos= (rand()%20)-9.5;
			
			initpos=RVO::Vector2(xpos,ypos);
			sim->setAgentPosition(i,initpos);
		/*	do{
				xgpos= rand()%(9);
				ygpos= rand()%(20);
				tempgoal=RVO::Vector2(xgpos,ygpos);
			}while((RVO::abs(tempgoal-initpos)<((float)sidesize/(float)2))||((xgpos>=(((float)sidesize/(float)2)-3))&&(xgpos<=(((float)sidesize/(float)2)+3))&&(ygpos>=(((float)sidesize/(float)2)-3))&&(ygpos<=(((float)sidesize/(float)2)+3))));
			*/
		}while(!checkOverlap(i));  //||((xpos>=6)&&(xpos<=9)&&(ypos<=(sidesize-6))&&(ypos>=(sidesize-9))) ||((xpos>=16)&&(xpos<=19)&&(ypos<=(sidesize-16))&&(ypos>=(sidesize-19))));// ||((xpos>=4)&&(xpos<=5)&&(ypos<=(sidesize-15))&&(ypos>=(sidesize-17))) ||((xpos>=13)&&(xpos<=17)&&(ypos<=(sidesize-4))&&(ypos>=(sidesize-6)))   );
		
		
		initpositions << sim->getAgentPosition(i).x() << "\n"<< sim->getAgentPosition(i).y() << "\n" << xgpos <<"\n" << ygpos<<"\n";
		//std::cout << "("<<i<<") "<< sim->getAgentPosition(i).x() << " "<< sim->getAgentPosition(i).y() << " " << xgpos <<" " << ygpos<<"\n";
		
		//goals.push_back(RVO::Vector2(xgpos,ygpos));
		
		 //sim->setAgentGoal(i, RVO::Vector2(xgpos,ygpos));
		
		}
		
 	initpositions.close();
 	//std::cout << "Agents generated.." << iteration << "\n";
}


void getScenario()  //Reads the position of the agents in the Crowd scenario, for a previously generated environment.
{
    std::string line;
    int i=0,pos1,pos2,goal1,goal2;
    std::ifstream initpositions;
    initpositions.open("positions.txt", std::ios_base::app);
    
    for (int k=0; k<Agents; k++)
    {
        
        sim->addAgent(RVO::Vector2());
	}
    
    if (initpositions.is_open())
    {
        while ( getline (initpositions,line) )
        {
            pos1=atoi(line.c_str());
            getline(initpositions,line);
            pos2=atoi(line.c_str());
            getline(initpositions,line);
            goal1=atoi(line.c_str());
            getline(initpositions,line);
            goal2=atoi(line.c_str());
            sim->setAgentPosition(i, RVO::Vector2(pos1,pos2));
            goals.push_back(RVO::Vector2(goal1,goal2));
            
            
            i=i+1;
            
          //  std::cout << "Adding agent " << i << "\n";
        }
        initpositions.close();
    }
    
    else std::cout << "Unable to open file";
    
}

void simulateVelocities()  //Simulates the execution of each action/velocity for a number of timesteps (defined in timeHorizon) in the future.
{
    
    sim->buildTree();
    sim->setVvalues();
    
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i)
    {
        
        if(needtoUpdate[i])
        {
            float finalrw,finalaction=0;
            
            
            for (int a=0;a<Actions; a++)
            {   RewardAvg[i][a]=0;
                RewardAvg[i][a] = sim->SimulateVel(i, goals[i], timeHorizon, a,numNeighbors, Agents,  coord_factor, ActionSpeed[i][a],ActionDir[i][a]);
                
            }
            
            finalrw=-1000;
            
            for (int ac=0;ac<Actions; ac++)
            {
				if(i==0)
				{
				//std::cout << " Reward for action " << ac << " is "<< RewardAvg[i][ac] << "\n";	
					
				}
                
                if(RewardAvg[i][ac]>=(finalrw))
                {
                    finalrw=RewardAvg[i][ac];
                    finalaction=ac;
                    
                }
                
            }
            
            chosenAction[i]=finalaction;

            if(sim->getAgentNumAgentNeighbors(i)==0)
            {

            	//chosenAction[i]=0;
            }
            
        }
        
    }
}

void setOptimization(int i)
{
    float MyProg;
    Selfish[i][chosenAction[i]]=(RVO::normalize(goalvectors[i]) *sim->getAgentVelocity(i))/(float)1.5; //
    Polite[i][chosenAction[i]]=(sim->getAgentVelocity(i)*sim->getAgentPrefVelocity(i))/(float)2.25;//1-RVO::abs(sim->getAgentPrefVelocity(i)-sim->getAgentVelocity(i))/(float)3;
    MyProg=(1-coord_factor)*Selfish[i][chosenAction[i]]+coord_factor*Polite[i][chosenAction[i]];
    
    totalGoalOriented[i]=totalGoalOriented[i]+Selfish[i][chosenAction[i]];
    
    realProg[i] = (MyProg);
}

void setActions(int i)  //Sets the actions for each agent
{
    int  o;
    float novector, fullvector;
    for(o=0;o<Actions;o++)
    {
        ActionSpeed[i][o]=actionVectorMag[o];
        ActionDir[i][o]= actionVector[o];
         goalvectors[i]= RVO::normalize(goals[i] - sim->getAgentPosition(i));
          RVO::Vector2 pVel = RVO::Vector2((RVO::normalize(goalvectors[i])*ActionSpeed[i][o]).x()*std::cos(ActionDir[i][o])+(goalvectors[i]*ActionSpeed[i][o]).y()*std::sin(ActionDir[i][o]), (goalvectors[i]*ActionSpeed[i][o]).y()*std::cos(ActionDir[i][o])+(goalvectors[i]*ActionSpeed[i][o]).x()*-std::sin(ActionDir[i][o])); 
		  novector = (1-coord_factor)*(RVO::Vector2(0,0)*goalvectors[i])/(float)1.5 +coord_factor*(RVO::Vector2(0,0)*pVel)/(float)2.25 ;
		  fullvector= (1-coord_factor)*(pVel*goalvectors[i])/(float)1.5 +coord_factor*(pVel*pVel)/(float)2.25 ;
		  
		  		// defaultValue[i][o]= (1-coord_factor)*(pVel*goalvectors[i])/(float)1.5 +coord_factor*(pVel*pVel)/(float)2.25 ;
		  		 if(fullvector>novector)
		  		 {
					defaultValue[i][o]= fullvector;
					}
					else
					{
						defaultValue[i][o]= novector;
					}
		  		 
		  		 if(i==agentViewed)//(!isinGoal[i])//(i==agentViewed)
		  		 {
					 std::cout << "Default value action " << o << " is " << defaultValue[i][o] << "\n";
					 
					 
					 
				}
		  		 
    }
    float suma=0;
    for (o=0; o<Actions; o++) 
	{  //  Qvalues[i][j]= LastActionEstimate[i][j];
		//suma=suma+exp(defaultValue[i][o]/(float).05);
		
		suma=suma+defaultValue[i][o];
		
		
	}
	
	if (i==agentViewed)
		  		 {
					 std::cout << "SUMA " << suma << " AVG " << suma/(float)Actions << "\n";

				}
	
	for (o=0; o<Actions; o++) 
	{
		Boltz[i][o]=100*(exp(defaultValue[i][o]/(float).05))/suma;
		
		 if (i==0)
		  		 {
				//	 std::cout << "Boltz action " << o << " is " << Boltz[i][o] << "\n";

				}
		
	}
    

    
}

void setPreferredVelocities() //Computes the preferred velocity of the agents based on the action chosen (chosenAction[])
{
    
    
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i)
    {
        /*
         * Set the preferred velocity to be a vector of unit magnitude (speed) in the
         * direction of the goal.
         */
        
        if(isinGoal[i]==0) //If agent i has not reached its goal
        {
            
            
            
            if(scenario==5)
            {
                RVO::Vector2 finalGoal;
                RVO::Vector2 goalPos;
                
                
                finalGoal = RVO::Vector2(-30, 0.0);
                goalPos = RVO::Vector2(-10, 0.0);
                
                
                if (sim->queryVisibility(sim->getAgentPosition(i), finalGoal, sim->getAgentRadius(i)))
                {
                    goalPos = finalGoal;
                    
                }
                else
                {
                    goalPos = RVO::Vector2(-10, 0.0);
                }
                
                
                goalvectors[i]= RVO::normalize( goalPos - sim->getAgentPosition(i));
                goaldistance[i]= RVO::abs(  goalPos - sim->getAgentPosition(i));
                
            }
            
            if(scenario==9)
            {
                RVO::Vector2 finalGoal;
                RVO::Vector2 goalPos;
                
                
                finalGoal = RVO::Vector2(-initPos[i].x(), initPos[i].y());
                
                 if(i<10)
            {
           
                
                goalPos = RVO::Vector2(1, 0);
           }
           else
           {
			    goalPos = RVO::Vector2(-1, 0);
			  }     
                
                if (sim->queryVisibility(sim->getAgentPosition(i), finalGoal, sim->getAgentRadius(i)))
                {
                    goalPos = finalGoal;
                    
                }
               /* else
                {
                    goalPos = RVO::Vector2(-10, 0.0);
                }
                */
                
                goalvectors[i]= RVO::normalize( goalPos - sim->getAgentPosition(i));
                goaldistance[i]= RVO::abs(  goalPos - sim->getAgentPosition(i));
                
            }
            
            
             if((scenario!=9)&&(scenario!=5))
            {
                goaldistance[i]=RVO::abs(goals[i] - sim->getAgentPosition(i));
                goalvectors[i]= RVO::normalize(goals[i] - sim->getAgentPosition(i));
                
            }
            
            
            goalx[i]=goalvectors[i].x();
            goaly[i]=goalvectors[i].y();
            
            sim->setAgentPrefVelocity(i, goalvectors[i]*ActionSpeed[i][chosenAction[i]]);
            
            if(algo==1)
            {
                
                
                angle = std::rand() * 2.0f * M_PI / RAND_MAX;
                dist = std::rand() * 0.01f / RAND_MAX;
                
                sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) + dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
                
            }
            
            
            
            
            if(algo!=1)
            {
                angle =  ActionDir[i][chosenAction[i]];
                dist = std::rand() * 0.01f / RAND_MAX;
                sim->setAgentPrefVelocity(i, RVO::Vector2(sim->getAgentPrefVelocity(i).x()*std::cos(angle)+sim->getAgentPrefVelocity(i).y()*std::sin(angle),  sim->getAgentPrefVelocity(i).y()*std::cos(angle)+sim->getAgentPrefVelocity(i).x()*-std::sin(angle)) );
                
                
                /*
                 * Perturb a little to avoid deadlocks due to perfect symmetry.
                 */
                if(randomPert)
                {
                    angle = std::rand() * 2.0f * M_PI / RAND_MAX;
                    
                    if(i==agentViewed)
                    {
                   // std::cout << " CurrentVpref: " << sim->getAgentPrefVelocity(i) ;
				    }
				    
				    
                    sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +  dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
                    
                     if(i==agentViewed)
                    {
                     //std::cout << " NewVpref: " << sim->getAgentPrefVelocity(i)  << " \n"  ;
				    } 
                    
                    
                }
                
                
            }
            
            
        }
        else  //if agent i has reached its goal, move it to position (-1000, -1000) and stop it
        {
            sim->setAgentPosition(i,RVO::Vector2(-1000.0f,-1000.0f));
            sim->setAgentVelocity(i, RVO::Vector2());
            
        }
        
        
      
        
    }
    
    
    
}

void setupScenario(RVO::RVOSimulator* sim) //Initialize the Simulation, positions of the obstacles and the agents
{
    /* Specify the global time step of the simulation. */
    sim->setTimeStep(simTimeStep);
   
    int j;
    totalingoal=0;
    timestep=0;
    totalrewardsimulation=0;
    globalAvgReward=0;  
    globalGoalOriented=0;

    
    
    for (int i = 0; i < Agents; ++i)
    {
    	cumReward[i]=0;
    	decisionSteps=0;
        lastReward[i]=0;
        changeAction[i]=0;
        lastAction[i]=0;
        needtoUpdate[i]=0;
        sim->setAgentDefaults(neighborDistance, numNeighbors, timeHorizonORCA , timeHorizonObstORCA , radiusORCA , maxSpeedORCA);
        chosenAction[i]=0;
        epsilon[i]=100;
        TimetoGoal[i]=0;
        temp[i]=0.2;
        UCBconst[i]=1.4f;
        
        isinGoal[i]=0;
        
        
        
        
        for(j=0;j<Actions;j++)
        {
            RewardAvg[i][j]=0;
           
            Qvalues[i][j]=0;
            UCBound[i][j]=0;
            Chosen[i][j]=0;
            LastActionEstimate[i][j] =  Qvalues[i][j];
        }
        
        
        if(algo==1)
        {
            ActionSpeed[i][0]=1.5;
            ActionDir[i][0]=0;
            chosenAction[i]=0;
        }
        
        
        
        
        
    }
    
    /*
     * Adding Obstacles for each scenario
     *
     */
    
    
    std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4,obstacle5, obstacle6, obstacle7, obstacle8;
    
    
    if(scenario==3)
    {
        obstacle1.push_back(RVO::Vector2(200.0f,   31.0f));
        obstacle1.push_back(RVO::Vector2(-200.0f,   31.0f));
        obstacle1.push_back(RVO::Vector2(-200.0f, 30.6f));
        obstacle1.push_back(RVO::Vector2(200.0f, 30.6f));
        
        obstacle2.push_back(RVO::Vector2(200.0f,   27.4f));
        obstacle2.push_back(RVO::Vector2(-200.0f, 27.4f));
        obstacle2.push_back(RVO::Vector2(-200.0f, 27.0f));
        obstacle2.push_back(RVO::Vector2(200.0f, 27.0f));
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
        sim->processObstacles();
        
    }
    
  if(scenario==4)
  {
	  
	  obstacle1.push_back(RVO::Vector2(200.0f,   3.0f));
        obstacle1.push_back(RVO::Vector2(2.0f,   3.0f));
        obstacle1.push_back(RVO::Vector2(2.0f, 2.0f));
        obstacle1.push_back(RVO::Vector2(200.0f, 2.0f));
        
        obstacle2.push_back(RVO::Vector2(200.0f,   -2.0f));
        obstacle2.push_back(RVO::Vector2(2.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(2.0f, -3.0f));
        obstacle2.push_back(RVO::Vector2(200.0f, -3.0f));
        
        
        
        obstacle3.push_back(RVO::Vector2(-2.0f,   3.0f));
        obstacle3.push_back(RVO::Vector2(-200.0f,   3.0f));
        obstacle3.push_back(RVO::Vector2(-200.0f, 2.0f));
        obstacle3.push_back(RVO::Vector2(-2.0f, 2.0f));
        
        obstacle4.push_back(RVO::Vector2(-2.0f, -2.0f));
        obstacle4.push_back(RVO::Vector2(-200.0f,   -2.0f));
        obstacle4.push_back(RVO::Vector2(-200.0f, -3.0f));
        obstacle4.push_back(RVO::Vector2(-2.0f, -3.0f));
        
        
        
        
	    obstacle5.push_back(RVO::Vector2(3.0f,200.0f));
        obstacle5.push_back(RVO::Vector2(2.0f, 200.0f));
        obstacle5.push_back(RVO::Vector2(2.0f, 3.0f));
        obstacle5.push_back(RVO::Vector2(3.0f, 3.0f));
        
        obstacle6.push_back(RVO::Vector2(-2.0f,   200.0f));
        obstacle6.push_back(RVO::Vector2(-3.0f, 200.0f));
        obstacle6.push_back(RVO::Vector2(-3.0f, 3.0f));
        obstacle6.push_back(RVO::Vector2(-2.0f, 3.0f));
        
        
        
        obstacle7.push_back(RVO::Vector2(3.0f,   -3.0f));
        obstacle7.push_back(RVO::Vector2(2.0f,   -3.0f));
        obstacle7.push_back(RVO::Vector2(2.0f, -200.0f));
        obstacle7.push_back(RVO::Vector2(3.0f, -200.0f));
        
        obstacle8.push_back(RVO::Vector2(-2.0f, -3.0f));
        obstacle8.push_back(RVO::Vector2(-3.0f,   -3.0f));
        obstacle8.push_back(RVO::Vector2(-3.0f, -200.0f));
        obstacle8.push_back(RVO::Vector2(-2.0f, -200.0f));
        
        
	  
	  /* obstacle1.push_back(RVO::Vector2(0.5f,   350.0f));
        obstacle1.push_back(RVO::Vector2(-0.5f,   350.0f));
        obstacle1.push_back(RVO::Vector2(-0.5f, 0.6f));
        obstacle1.push_back(RVO::Vector2(0.5f, 0.6f));
        
        obstacle2.push_back(RVO::Vector2(0.5f,   -0.6f));
        obstacle2.push_back(RVO::Vector2(-0.5f, -0.6f));
        obstacle2.push_back(RVO::Vector2(-0.5f, -150.0f));
        obstacle2.push_back(RVO::Vector2(0.5f, -150.0f));
        * */
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        
         sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        sim->addObstacle(obstacle7);
        sim->addObstacle(obstacle8);
        
        sim->processObstacles(); 
	  
	 }  
    
    if(scenario==5)
    {
        
        
        obstacle1.push_back(RVO::Vector2(-10.0f,   350.0f));
        obstacle1.push_back(RVO::Vector2(-11.0f,   350.0f));
        obstacle1.push_back(RVO::Vector2(-11.0f, 0.7f));
        obstacle1.push_back(RVO::Vector2(-10.0f, 0.7f));
        
        obstacle2.push_back(RVO::Vector2(-10.0f,   -0.7f));
        obstacle2.push_back(RVO::Vector2(-11.0f, -0.7f));
        obstacle2.push_back(RVO::Vector2(-11.0f, -150.0f));
        obstacle2.push_back(RVO::Vector2(-10.0f, -150.0f));
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
        sim->processObstacles();
    }
    
    
   /*   if(scenario==6)
    {
        
        
        obstacle1.push_back(RVO::Vector2(-4.1f,   350.0f));
        obstacle1.push_back(RVO::Vector2(-5.1f,   350.0f));
        obstacle1.push_back(RVO::Vector2(-5.1f, -100.6f));
        obstacle1.push_back(RVO::Vector2(-4.1f, -100.6f));
        
        obstacle2.push_back(RVO::Vector2(-0.9f,   350.6f));
        obstacle2.push_back(RVO::Vector2(-1.9f, 350.6f));
        obstacle2.push_back(RVO::Vector2(-1.9f, -100.0f));
        obstacle2.push_back(RVO::Vector2(-0.9f, -100.0f));
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
        sim->processObstacles();
    }*/
    
    if(scenario==7)
    {
        obstacle1.push_back(RVO::Vector2(-10.0f,   2.0f));
        obstacle1.push_back(RVO::Vector2(-10.0f,   0.6f));
        obstacle1.push_back(RVO::Vector2(10.0f, 0.6f));
        obstacle1.push_back(RVO::Vector2(10.0f, 2.0f));
        
        obstacle2.push_back(RVO::Vector2(-10.0f,   -0.6f));
        obstacle2.push_back(RVO::Vector2(-10.0f,   -2.0f));
        obstacle2.push_back(RVO::Vector2(10.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(10.0f, -0.6f));
        
        obstacle3.push_back(RVO::Vector2(-100.0f,   100.0f));
        obstacle3.push_back(RVO::Vector2(-100.0f,   10.0f));
        obstacle3.push_back(RVO::Vector2(-10.0f, 0.6f));
        obstacle3.push_back(RVO::Vector2(-10.0f, 100.0f));
        
         obstacle4.push_back(RVO::Vector2(-100.0f,   -10.0f));
        obstacle4.push_back(RVO::Vector2(-100.0f,   -100.0f));
        obstacle4.push_back(RVO::Vector2(-10.0f, -100.0f));
        obstacle4.push_back(RVO::Vector2(-10.0f, -0.6f));
        
        
        obstacle5.push_back(RVO::Vector2(10.0f,   100.0f));
        obstacle5.push_back(RVO::Vector2(10.0f,   0.6f));
        obstacle5.push_back(RVO::Vector2(100.0f, 10.0f));
        obstacle5.push_back(RVO::Vector2(100.0f, 100.0f));
        
         obstacle6.push_back(RVO::Vector2(10.0f,   -0.6f));
        obstacle6.push_back(RVO::Vector2(10.0f,   -100.0f));
        obstacle6.push_back(RVO::Vector2(100.0f, -100.0f));
        obstacle6.push_back(RVO::Vector2(100.0f, -10.0f));
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
         sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        
          sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        
        sim->processObstacles();
    }
    
    if(scenario==8)
    {
		
		/*
		obstacle1.push_back(RVO::Vector2(-2.0f,   2.0f));
        obstacle1.push_back(RVO::Vector2(0.0f,   -2.0f));
        obstacle1.push_back(RVO::Vector2(2.0f, -2.0f));
        obstacle1.push_back(RVO::Vector2(2.0f, 2.0f));
        
        sim->addObstacle(obstacle1);
        sim->processObstacles();
		*/
		/*
		obstacle1.push_back(RVO::Vector2(-1.0f,   0.5f));
        obstacle1.push_back(RVO::Vector2(-1.0f,   -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, 0.5f));
        
        obstacle2.push_back(RVO::Vector2(3.0f,   3.5f));
        obstacle2.push_back(RVO::Vector2(3.0f,   1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 3.5f));
        
        obstacle3.push_back(RVO::Vector2(-4.5f,   -2.0f));
        obstacle3.push_back(RVO::Vector2(-4.5f,   -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -2.0f));
        
        obstacle4.push_back(RVO::Vector2(-1.5f,   6.5f));
        obstacle4.push_back(RVO::Vector2(-1.5f,   4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 6.5f));
        
        
         
  
		obstacle5.push_back(RVO::Vector2(3.0f,   0.0f));
        obstacle5.push_back(RVO::Vector2(3.0f,   -2.0f));
        obstacle5.push_back(RVO::Vector2(5.0f, -2.0f));
        obstacle5.push_back(RVO::Vector2(5.0f, 0.0f));
        
        obstacle6.push_back(RVO::Vector2(7.0f,   3.5f));
        obstacle6.push_back(RVO::Vector2(7.0f,   1.5f));
        obstacle6.push_back(RVO::Vector2(9.0f, 1.5f));
        obstacle6.push_back(RVO::Vector2(9.0f, 3.5f));
        
        obstacle7.push_back(RVO::Vector2(-0.5f,   -3.0f));
        obstacle7.push_back(RVO::Vector2(-0.5f,   -5.0f));
        obstacle7.push_back(RVO::Vector2(1.5f, -5.0f));
        obstacle7.push_back(RVO::Vector2(1.5f, -3.0f));
        
        obstacle8.push_back(RVO::Vector2(2.5f,   7.0f));
        obstacle8.push_back(RVO::Vector2(2.5f,   5.0f));
        obstacle8.push_back(RVO::Vector2(4.5f, 5.0f));
        obstacle8.push_back(RVO::Vector2(4.5f, 7.0f));				      
                        
        
        
        
                          
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        
        sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        sim->addObstacle(obstacle7);
        sim->addObstacle(obstacle8);
        
        
        sim->processObstacles();
        * */
        
        obstacle1.push_back(RVO::Vector2(-1.0f,   0.5f));
        obstacle1.push_back(RVO::Vector2(-1.0f,   -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, 0.5f));
        
        obstacle2.push_back(RVO::Vector2(3.0f,   3.5f));
        obstacle2.push_back(RVO::Vector2(3.0f,   1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 3.5f));
        
        obstacle3.push_back(RVO::Vector2(-4.5f,   -2.0f));
        obstacle3.push_back(RVO::Vector2(-4.5f,   -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -2.0f));
        
         obstacle4.push_back(RVO::Vector2(-1.5f,   6.5f));
        obstacle4.push_back(RVO::Vector2(-1.5f,   4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 6.5f));
        
  
                          
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        sim->processObstacles();
        
	}
    
    
    
    if(scenario==1)
    {
        
        
        obstacle1.push_back(RVO::Vector2(-1.5f,   -0.5f));
        obstacle1.push_back(RVO::Vector2(-1.5f,   -1.5f));
        obstacle1.push_back(RVO::Vector2(sidesize+1.5f, -1.5f));
        obstacle1.push_back(RVO::Vector2(sidesize+1.5f, -0.5f));
        
        obstacle2.push_back(RVO::Vector2(sidesize+0.5f,   sidesize+1.5f));
        obstacle2.push_back(RVO::Vector2(sidesize+0.5f,   -1.5f));
        obstacle2.push_back(RVO::Vector2(sidesize+1.5f, -1.5f));
        obstacle2.push_back(RVO::Vector2(sidesize+1.5f, sidesize+1.5f));
        
        obstacle3.push_back(RVO::Vector2(-1.5f,   sidesize+1.5f));
        obstacle3.push_back(RVO::Vector2(-1.5f,   sidesize+0.5f));
        obstacle3.push_back(RVO::Vector2(sidesize+1.5f,  sidesize+0.5f));
        obstacle3.push_back(RVO::Vector2(sidesize+1.5f,  sidesize+1.5f));
        
        obstacle4.push_back(RVO::Vector2(-1.5f,   sidesize+1.5f));
        obstacle4.push_back(RVO::Vector2(-1.5f,   -1.5f));
        obstacle4.push_back(RVO::Vector2(-0.5f, -1.5f));
        obstacle4.push_back(RVO::Vector2(-0.5f, sidesize+1.5f));
      /*  
        obstacle5.push_back(RVO::Vector2((((float)sidesize/(float)2)-3),   (((float)sidesize/(float)2)+3)));
        obstacle5.push_back(RVO::Vector2((((float)sidesize/(float)2)-3),   (((float)sidesize/(float)2)-3)));
        obstacle5.push_back(RVO::Vector2((((float)sidesize/(float)2)+3), (((float)sidesize/(float)2)-3)));
        obstacle5.push_back(RVO::Vector2((((float)sidesize/(float)2)+3),(((float)sidesize/(float)2)+3)));
        */
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
       // sim->addObstacle(obstacle5);
        
        sim->processObstacles();
        
      
        
        if(iteration>1)
        {
            getScenario();
            
        }
        
        if(iteration==1)
        {
            genScenario();
        }
        
    }
    
    //Adding agents for each scenario
    
    if(scenario==2) //Circle Scenario
    {
        
        for (int i = 0; i < Agents; ++i)
        {
            sim->addAgent(float(Agents)/(float)1 *
                          RVO::Vector2(std::cos(i * 2.0f * M_PI / float(Agents))+(std::rand() * 0.01f /(float)RAND_MAX) , std::sin(i * 2.0f * M_PI / float(Agents))+(std::rand() * 0.01f /(float)RAND_MAX) ));
            
            goals.push_back(-sim->getAgentPosition(i));
        }
    }
    
    if(scenario==3)//Bidirectional Flow
    {
        
        
        sim->addAgent(RVO::Vector2(-20.0f,  30.0f));
        sim->addAgent(RVO::Vector2(-17.0f,  30.0f));
        sim->addAgent(RVO::Vector2(-14.0f,  30.0f));
        sim->addAgent(RVO::Vector2(-20.0f,  29.0f));
        sim->addAgent(RVO::Vector2(-17.0f,  29.0f));
        
        sim->addAgent(RVO::Vector2(-14.0f,  29.0f));
        sim->addAgent(RVO::Vector2(-20.0f,  28.0f));
        sim->addAgent(RVO::Vector2(-17.0f,  28.0f));
        sim->addAgent(RVO::Vector2(-14.0f,  28.0f));
        sim->addAgent(RVO::Vector2(20.0f,  30.0f));
        
        sim->addAgent(RVO::Vector2(17.0f,  30.0f));
        sim->addAgent(RVO::Vector2(14.0f,  30.0f));
        sim->addAgent(RVO::Vector2(20.0f,  29.0f));
        sim->addAgent(RVO::Vector2(17.0f,  29.0f));
        sim->addAgent(RVO::Vector2(14.0f,  29.0f));
        
        sim->addAgent(RVO::Vector2(20.0f,  28.0f));
        sim->addAgent(RVO::Vector2(17.0f,  28.0f));
        sim->addAgent(RVO::Vector2(14.0f,  28.0f));
        
        
        
        
        
        
        for (int i = 0; i < Agents; ++i)
        {
            goals.push_back(RVO::Vector2(-sim->getAgentPosition(i).x(),sim->getAgentPosition(i).y() ) );
            
        }
    }
    
    
    if(scenario==4)
    {
        
        
        sim->addAgent(RVO::Vector2(-30.0f,  0.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  0.5f));
        sim->addAgent(RVO::Vector2(-22.0f,   0.5f));
        sim->addAgent(RVO::Vector2(-18.0f,   0.5f));
        sim->addAgent(RVO::Vector2(-14.0f,   0.5f));
        
        
        sim->addAgent(RVO::Vector2(-30.0f, -0.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(-22.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(-18.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(-14.0f,  -0.5f));
        
          sim->addAgent(RVO::Vector2(-30.0f, -1.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-22.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-18.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-14.0f,  -1.5f));
        
         sim->addAgent(RVO::Vector2(-30.0f,  1.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  1.5f));
        sim->addAgent(RVO::Vector2(-22.0f,   1.5f));
        sim->addAgent(RVO::Vector2(-18.0f,   1.5f));
        sim->addAgent(RVO::Vector2(-14.0f,   1.5f));
   /*     
        sim->addAgent(RVO::Vector2(-32.0f,  1.5f));
        sim->addAgent(RVO::Vector2(-28.0f,  1.5f));
        sim->addAgent(RVO::Vector2(-24.0f,   1.5f));
        sim->addAgent(RVO::Vector2(-20.0f,   1.5f));
        sim->addAgent(RVO::Vector2(-16.0f,   1.5f));
        
           sim->addAgent(RVO::Vector2(-32.0f, -1.5f));
        sim->addAgent(RVO::Vector2(-28.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-24.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-20.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-16.0f,  -1.5f));
        
      */  
        
        
        
        sim->addAgent(RVO::Vector2(30.0f,  0.5f));
        sim->addAgent(RVO::Vector2(26.0f,  0.5f));
        sim->addAgent(RVO::Vector2(22.0f,  0.5f));
        sim->addAgent(RVO::Vector2(18.0f,  0.5f));
        sim->addAgent(RVO::Vector2(14.0f,  0.5f));
        
        
        sim->addAgent(RVO::Vector2(30.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(26.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(22.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(18.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(14.0f,  -0.5f));
        
         sim->addAgent(RVO::Vector2(30.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(26.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(22.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(18.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(14.0f,  -1.5f));
        
        sim->addAgent(RVO::Vector2(30.0f,  1.5f));
        sim->addAgent(RVO::Vector2(26.0f,  1.5f));
        sim->addAgent(RVO::Vector2(22.0f,  1.5f));
        sim->addAgent(RVO::Vector2(18.0f,  1.5f));
        sim->addAgent(RVO::Vector2(14.0f,  1.5f));
      /*  
        sim->addAgent(RVO::Vector2(32.0f,  1.5f));
        sim->addAgent(RVO::Vector2(28.0f,  1.5f));
        sim->addAgent(RVO::Vector2(24.0f,  1.5f));
        sim->addAgent(RVO::Vector2(20.0f,  1.5f));
        sim->addAgent(RVO::Vector2(16.0f,  1.5f));
        
           sim->addAgent(RVO::Vector2(32.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(28.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(24.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(20.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(16.0f,  -1.5f));
       */ 
        
        
        
        
        
        
        sim->addAgent(RVO::Vector2(0.5f,  30.0f));
        sim->addAgent(RVO::Vector2(0.5f,  26.0f));
        sim->addAgent(RVO::Vector2(0.5f,   22.0f));
        sim->addAgent(RVO::Vector2(0.5f,   18.0f));
        sim->addAgent(RVO::Vector2(0.5f,   14.0f));
        
        
       sim->addAgent(RVO::Vector2(-0.5f, 30.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  26.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  22.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  18.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  14.0f));
        
              sim->addAgent(RVO::Vector2(-1.5f, 30.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  26.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  22.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  18.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  14.0f));
       
        sim->addAgent(RVO::Vector2(1.5f,  30.0f));
        sim->addAgent(RVO::Vector2(1.5f,  26.0f));
        sim->addAgent(RVO::Vector2(1.5f,   22.0f));
        sim->addAgent(RVO::Vector2(1.5f,   18.0f));
        sim->addAgent(RVO::Vector2(1.5f,   14.0f));
        
    /*    
         sim->addAgent(RVO::Vector2(1.5f,  32.0f));
        sim->addAgent(RVO::Vector2(1.5f,  28.0f));
        sim->addAgent(RVO::Vector2(1.5f,   24.0f));
        sim->addAgent(RVO::Vector2(1.5f,   20.0f));
        sim->addAgent(RVO::Vector2(1.5f,   16.0f));
        
            sim->addAgent(RVO::Vector2(-1.5f, 32.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  28.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  24.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  20.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  16.0f));
      */  
        
        
        
        
        sim->addAgent(RVO::Vector2(0.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -14.0f));
        
        
        sim->addAgent(RVO::Vector2(-0.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -14.0f));
        
        
          sim->addAgent(RVO::Vector2(-1.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -14.0f));
        
        sim->addAgent(RVO::Vector2(1.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -14.0f));
       /* 
          sim->addAgent(RVO::Vector2(1.5f,  -32.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -28.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -24.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -20.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -16.0f));
        
         sim->addAgent(RVO::Vector2(-1.5f,  -32.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -28.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -24.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -20.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -16.0f));
        
        */
        
       /* sim->addAgent(RVO::Vector2(12.0f,  5.0f));
        
        sim->addAgent(RVO::Vector2(10.0f,  3.0f));
        sim->addAgent(RVO::Vector2(11.0f,  3.0f));
        sim->addAgent(RVO::Vector2(12.0f,  3.0f));
        
        
        
        sim->addAgent(RVO::Vector2(5.0f,  2.0f));
        sim->addAgent(RVO::Vector2(5.0f,  4.0f));
        */
        
        for (int i = 0; i < Agents; ++i)
        {
            initPos[i]=sim->getAgentPosition(i);
            
            if(i<(Agents/4)) //10
            {
                goals.push_back(RVO::Vector2(10, sim->getAgentPosition(i).y() ));
                
            }
            
            if((i>=(Agents/4))&&(i<(Agents/2)))  // 10 y 20
            {                
                  goals.push_back(RVO::Vector2(-10, sim->getAgentPosition(i).y() ));
                
            }
            
            if((i>=(Agents/2))&&(i<(Agents/((float)4/(float)3) ))) // 20 y 30
            {
                goals.push_back(RVO::Vector2(sim->getAgentPosition(i).x() ,-10));
                
            }
            
            
            if((i>=(Agents/((float)4/(float)3) ))&&(i<Agents))  // 30 y 40
            {
                goals.push_back(RVO::Vector2(sim->getAgentPosition(i).x() ,10));
                
            }
            
            
            
            
        }
        
    }
    
    if(scenario==5)
    { /*sim->addAgent( RVO::Vector2(-9.5f,  1.5f));
        sim->addAgent( RVO::Vector2(-9.5f,  2.5f));
        sim->addAgent( RVO::Vector2(-9.5f,  3.5f));
        sim->addAgent( RVO::Vector2(-9.5f,  4.5f));
		*/
        sim->addAgent( RVO::Vector2(-1.0f,  -1.5f));
        sim->addAgent( RVO::Vector2(-2.0f,  -4.5f));
        sim->addAgent( RVO::Vector2(-0.0f,  -4.5f));
        sim->addAgent( RVO::Vector2(-3.0f,  5.5f));
        sim->addAgent( RVO::Vector2(-1.0f,  6.5f));
        sim->addAgent( RVO::Vector2(-2.0f,  9.5f));
        sim->addAgent( RVO::Vector2(-2.0f,  2.5f));
        sim->addAgent( RVO::Vector2(1.0f,  -9.5f));
        sim->addAgent( RVO::Vector2(-3.0f,  -5.5f));
        
        sim->addAgent( RVO::Vector2(-4.0f,  1.5f));
        sim->addAgent( RVO::Vector2(-2.0f,  0.5f));
        sim->addAgent( RVO::Vector2(2.0f,  3.5f));
        sim->addAgent( RVO::Vector2(-3.0f,  2.5f));
        sim->addAgent( RVO::Vector2(-0.0f,  7.5f));
        sim->addAgent( RVO::Vector2(-1.0f,  9.5f));
        sim->addAgent( RVO::Vector2(-2.0f,  5.5f));
        
        sim->addAgent( RVO::Vector2(1.0f,  -7.0f));
        sim->addAgent( RVO::Vector2(1.0f,  -6.0f));
        sim->addAgent( RVO::Vector2(-3.0f,  -2.0f));
        sim->addAgent( RVO::Vector2(2.0f,  -8.0f));
        sim->addAgent( RVO::Vector2(-1.0f,  4.0f));
        sim->addAgent( RVO::Vector2(-0.0f,  6.0f));
        sim->addAgent( RVO::Vector2(1.0f,  -4.0f));
        sim->addAgent( RVO::Vector2(-4.0f,  0.0f));
        sim->addAgent( RVO::Vector2(5.0f,  4.0f));
        
        sim->addAgent( RVO::Vector2(1.0f,  -1.0f));
        sim->addAgent( RVO::Vector2(-1.0f,  -6.0f));
        sim->addAgent( RVO::Vector2(0.0f,  -3.0f));
        sim->addAgent( RVO::Vector2(2.0f,  -7.0f));
        sim->addAgent( RVO::Vector2(-2.0f,  4.0f));
        sim->addAgent( RVO::Vector2(0.0f,  0.0f));
        sim->addAgent( RVO::Vector2(1.0f,  8.0f));
        
        
        //genScenarioCong();
        for (int i = 0; i < Agents; ++i)
        {
            goals.push_back(RVO::Vector2(-10.0,0.0));
            
        }
        
    }
    
    if(scenario==6)
    {
        
        sim->addAgent( RVO::Vector2(-5.0f,  -10.0f));
        sim->addAgent(  RVO::Vector2(-4.0f,  -10.0f));
        sim->addAgent( RVO::Vector2(-3.0f,  -10.0f));
        sim->addAgent( RVO::Vector2(-2.0f,  -10.0f));
        sim->addAgent( RVO::Vector2(-1.0f,  -10.0f));
        
        sim->addAgent( RVO::Vector2(-5.0f,  -9.0f));
        sim->addAgent(  RVO::Vector2(-4.0f,  -9.0f));
        sim->addAgent( RVO::Vector2(-3.0f,  -9.0f));
        sim->addAgent(  RVO::Vector2(-2.0f,  -9.0f));
        sim->addAgent(  RVO::Vector2(-1.0f,  -9.0f));
        
        sim->addAgent( RVO::Vector2(-5.0f,  -8.0f));
        sim->addAgent(  RVO::Vector2(-4.0f,  -8.0f));
        sim->addAgent(  RVO::Vector2(-3.0f,  -8.0f));
        sim->addAgent( RVO::Vector2(-2.0f,  -8.0f));
        sim->addAgent(  RVO::Vector2(-1.0f,  -8.0f));
        
       /* 
        sim->addAgent( RVO::Vector2(-5.0f,  -7.0f));
        sim->addAgent(  RVO::Vector2(-4.0f,  -7.0f));
        sim->addAgent(  RVO::Vector2(-3.0f,  -7.0f));
        sim->addAgent( RVO::Vector2(-2.0f,  -7.0f));
        sim->addAgent(  RVO::Vector2(-1.0f,  -7.0f));
        
        sim->addAgent( RVO::Vector2(-5.0f,  -6.0f));
        sim->addAgent(  RVO::Vector2(-4.0f,  -6.0f));
        sim->addAgent(  RVO::Vector2(-3.0f,  -6.0f));
        sim->addAgent( RVO::Vector2(-2.0f,  -6.0f));
        sim->addAgent(  RVO::Vector2(-1.0f,  -6.0f));
        */
        
        
        sim->addAgent(  RVO::Vector2(-3.0f,  3.0f)); //14.0
        
        
        for (int i = 0; i < Agents-1; ++i)
        {
            goals.push_back(RVO::Vector2(sim->getAgentPosition(i).x(),24+sim->getAgentPosition(i).y() ) );
            
        }
        
        goals.push_back(RVO::Vector2(sim->getAgentPosition(15).x(),-20 ) );
        //std::cout<<" GOL : " << goals[4] <<"\n";
        
        
    }
    
    
    if(scenario==7)
    {
        
        sim->addAgent(  RVO::Vector2(-21.0f,  -0.0f));
        sim->addAgent( RVO::Vector2(21.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-23.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(23.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-25.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(25.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-27.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(27.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-29.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(29.0f,  0.0f));
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[0]=RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) ;
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[1]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[2]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[3]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[4]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[5]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[6]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[7]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        goals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        goals[8]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        goals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        goals[9]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        
        
    }
    
    
    if(scenario==8)
    {
		/*
	sim->addAgent(  RVO::Vector2(-5.0f,  0.0f));	
	
	goals.push_back(RVO::Vector2(-sim->getAgentPosition(0).x(),sim->getAgentPosition(0).y() ) );
		*/
		sim->addAgent(  RVO::Vector2(-15.0f,  0.0f));	
	sim->addAgent(  RVO::Vector2(-15.0f,  5.0f));	
	sim->addAgent(  RVO::Vector2(-15.0f,  -1.0f));	
	sim->addAgent(  RVO::Vector2(-15.0f,  2.5f));	
	sim->addAgent(  RVO::Vector2(-15.0f,  -2.5f));	
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(0).y() ) );
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(1).y() ) );
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(2).y() ) );
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(3).y() ) );
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(4).y() ) );
		
		/*
		sim->addAgent(  RVO::Vector2(-5.0f,  0.0f));	
	sim->addAgent(  RVO::Vector2(-5.0f,  5.0f));	
	sim->addAgent(  RVO::Vector2(-5.0f,  -1.0f));	
	sim->addAgent(  RVO::Vector2(-5.0f,  2.5f));	
	sim->addAgent(  RVO::Vector2(-5.0f,  -2.5f));	
	
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(0).y() ) );
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(1).y() ) );
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(2).y() ) );
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(3).y() ) );
	goals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(4).y() ) );
	*/
	
	}
    
    for (int i = 0; i < Agents; ++i)
    {
        
        setActions(i);
        sim->setAgentinGoal(i,0);
        
        for (j=0; j<sizeTW; j++)
        {
            TimeWindow[i][j]=-1000;
            TWscore[i][j]=-1000;
        }
        
        
    }
    
    
    setPreferredVelocities();
    
       float orcaavgtime=0;
    for (int i = 0; i < Agents; ++i)
    {
		orcaavgtime=orcaavgtime-0.3 + RVO::abs(goals[i]-sim->getAgentPosition(i))/(float)1.5;
		
		sim->setAgentVelocity(i,sim->getAgentPrefVelocity(i));
		
	}
	
	orcaavgtime=orcaavgtime/(float)Agents;
	
	float tempstandard=0;
	for (int i = 0; i < Agents; ++i)
    {
		//std::cout << " Agent " << i << " " << -0.3+ RVO::abs(goals[i]-sim->getAgentPosition(i))/(float)1.5<<"\n";
	 tempstandard=tempstandard+ ((  (RVO::abs(goals[i]-sim->getAgentPosition(i))/(float)1.5)  -orcaavgtime-0.3)*(  (RVO::abs(goals[i]-sim->getAgentPosition(i))/(float)1.5)   -orcaavgtime-0.3));
	 
	 ori[i] = atan2(sim->getAgentVelocity(i).y(),sim->getAgentVelocity(i).x());
    }
    
  //  std::cout << " ORCA AVG: " <<  orcaavgtime  << " STD " << tempstandard << "\n";
    if(Agents>1)
    {
      tempstandard=tempstandard/(float)(Agents-1);
			}
			else
			{
			 tempstandard=0;
			
			}
            tempstandard=sqrt(tempstandard);
				
            orcaavgtime=orcaavgtime+3*tempstandard;
       //    std::cout << " Min theoretical is " << orcaavgtime <<"\n";
    baseScore=orcaavgtime;
    
}

int maxActionEstimate(int i)
{
    int j,jmax, indexCand=0;
    float currentmax=-1000;
     float candidateUCB[10];
    int candidateAction[10];
    
    for (j=0; j<Actions; j++)
    {
		Qvalues[i][j]= LastActionEstimate[i][j];
	  
        
        
        if ( Qvalues[i][j]>=currentmax)
        {
            currentmax=Qvalues[i][j];
            
            jmax=j;
        }
        
    }
    
    if((currentmax<0.001)&&(currentmax>-0.001)) 
    {
	 for (j=0; j<Actions; j++)
    {
		
        if ( Qvalues[i][j]>-0.001)
        {
            currentmax=Qvalues[i][j];
            candidateAction[indexCand]=j;
            candidateUCB[indexCand]=Qvalues[i][j];
            indexCand=indexCand+1;
            
            
            
            
           
        }
		
		
	}
	 if(indexCand>0.5)
    {
		float probability= rand()%indexCand;
		    if(i==agentViewed)
	    {
		std::cout << " MORETHAN ONE candidate \n  " ;	
		}
		if(i==agentViewed)
	    {
		std::cout << " Prob is " << probability << " of a total of " << indexCand << " candidates, chosen is " << candidateAction[(int)probability] << " \n" ;	
		}
	//	char y =getchar();
		return candidateAction[(int)probability];
	}
		
		
	}
    
    
    
    
    return jmax;
    
    
    
}

int maxUCB(int i)
{
    int j,jmax, indexCand=0;
    float currentmax=-1000;
    float nextVel;
    float candidateUCB[10];
    int candidateAction[10];
    for (j=0; j<Actions; j++)
    {
		
		
        if ((UCBound[i][j]>currentmax)&&(currentmax<100000))
        {
            currentmax=UCBound[i][j];
            
            
            jmax=j;
        }
      
        if (UCBound[i][j]>1000000)
        {
            currentmax=UCBound[i][j];
            candidateAction[indexCand]=j;
            candidateUCB[indexCand]=UCBound[i][j];
            indexCand=indexCand+1;
            
            if(i==agentViewed)
	    {
		std::cout << " INFINITE " << UCBound[i][j]<< " for action " << j << "\n  " ;	
		}
                                   
        }
        
               
        
    }
    
    if(indexCand>0.5)
    {
		float probability= rand()%indexCand;
		if(i==agentViewed)
	    {
		std::cout << " Prob is " << probability << " of a total of " << indexCand << " candidates, chosen is " << candidateAction[(int)probability] << " \n" ;	
		}
		//char y =getchar();
		return candidateAction[(int)probability];
	}
    
    
    return jmax;
    
}


float AvgRw(int i, int j)
{
	float AvgWeight=0.1;
	
	return (Qvalues[i][j]*AvgWeight+realProg[i]*(1-AvgWeight));
	
	
}

void updateTimeWindow(int i)
{
    int j,k,gamba=0.99;
  //  double cumavg=0;
    
	for (k=0;k<Actions; k++)
	{
	cumavg[k]=0;	
	}
    
    
    for(j=0;j<sizeTW;j++)
    {
        
        //if(currentVel[i]==TimeWindow[i][j]) //If it is the same as before
     //   {
            cumavg[TimeWindow[i][j]]=cumavg[TimeWindow[i][j]]+(TWscore[i][j]);
            
     //   }
        
        if(i==agentViewed)
        {
            
            //   std::cout << " Slot "<< j << " used by " << TimeWindow[i][j] << " with score " << TWscore[i][j] << "\n";
        }
        
        
    }
    
    
   // return cumavg;
    
    
}

//******** Check if agents have reached their destination
bool reachedGoal()
{
    /* Check if all agents have reached their goals. */
    int allingoal=1;
    totalnotingoal=0;
    
    
    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {
        
        float distToGoal;
        
        if(scenario!=9)
        {
        distToGoal = RVO::abs(sim->getAgentPosition(i)-goals[i]);
	}
	else
	{
		 distToGoal = RVO::abs(sim->getAgentPosition(i)-RVO::Vector2(-initPos[i].x(),initPos[i].y()));
		
	}
	
        if (((distToGoal > 0.5)&&(isinGoal[i]==0)&&(scenario!=5))||((scenario==5)&&(sim->getAgentPosition(i).x() >-10.0f) ))
        {
            //Agent is consireded to reach its goal if it is 0.5 or less meters from it
            isinGoal[i]=0;
            allingoal=0;
            totalnotingoal++;
            
            
        }
        else
        {
            if(!isinGoal[i])
            {
                
                totalingoal=totalingoal+1;
            }
            
            isinGoal[i]=1;
            sim->setAgentinGoal(i,1);
            
            if(TimetoGoal[i]==0)
            {
                TimetoGoal[i]=sim->getGlobalTime();
                
            //    std::cout << " AGent " << i << " in goal at " << TimetoGoal[i] << "\n" ;
            }
            
        }
        
    }
    
    if (allingoal==0)
    {
        
        return false;
    }
    
    return true;
    
}



void UCB_angle() // from ActionMCMC
{
    int randomaction;
    
    int i,l;
    
    
    for(i=0;i<Agents;i++)
    {
        if((needtoUpdate[i]))//&&((std::rand()/(float)RAND_MAX)<=0.5))
        {
            currentVel[i]=chosenAction[i];
            LastActionEstimate[i][currentVel[i]]=realProg[i];
            decisionSteps=decisionSteps+1;
            if(chosenAction[i]==0)
            {cumReward[i]=cumReward[i]+1;}//realProg[i];}


             if(i==agentViewed)
            {
                std::cout << " 1****Chosen "<< chosenAction[i]<< " \n";
                
                
            }
            
            currentVel[i]=chosenAction[i];
                     
        
			
           // LastActionEstimate[i][currentVel[i]]=(realProg[i]+1)/(float)2;
          //   Qvalues[i][currentVel[i]] = AvgRw(i,currentVel[i]);
            
            if(i==agentViewed)
            {
                std::cout << " 2 ****Chosen "<< currentVel[i] << " now  " <<  Chosen[i][currentVel[i]] << " \n";
                
                
            }
            
            
            
            
            int k;
            
            
            int newtimestep=timestep;
            
            updateTimeWindow(i);
            
			for(int k=0; k<Actions;k++)
			{
				if(Chosen[i][k]>0)
				{
			
			    Qvalues[i][k] =cumavg[k]/(Chosen[i][k]);
			//	Qvalues[i][k]	= defaultValue[i][k];
			//	LastActionEstimate[i][k]= defaultValue[i][k];
				}
				else
				{
				Qvalues[i][k]	= 0;// (defaultValue[i][k]-defaultValue[i][k])/(float)2;//0;// defaultValue[i][k]/(float)2;
				LastActionEstimate[i][k]= 0;// (defaultValue[i][k]-defaultValue[i][k])/(float)2;// 0;// defaultValue[i][k]/(float)2;
				}
				
				
				
				
			}
			
			
         
         

        chosenAction[i]=Boltzmann(i);
        if(i==agentViewed)
        {
			
			std::cout << "Boltzmann Chosen: " << chosenAction[i] << "\n";
          //char g = getchar();
	  }
       
           
        }
        
        
    }
    
}

//**** Update the agents in the simulation ****** //
void updateVisualization() //Displays the agents (if display is enabled), and calls the methods for the algorithm chosen (ORCA or C-NAV). It is called each timestep
{
	
	std::ofstream posangleagg;
	
	
	//totaltime.open("totaltime.txt", std::ios_base::app);
	posangleagg.open("Traject.txt", std::ios_base::app);
    
    timestep++;
    
    if(visualizer)
    {
        glLineWidth(2);
        
    }
    
    if (lastFrameTime == 0)
    {
        if(visualizer)
        {
            lastFrameTime = glutGet(GLUT_ELAPSED_TIME);
        }
        
    }
    
    
    if(visualizer)
    {
        lastFrameTime = timestep;
    }
    else
    {
        lastFrameTime = timestep;
    }
    
    
    if((!finalize)) //To prevent iterations from occuring when the iteration is setting up
    {
        if ((!reachedGoal())&&(timestep<threshold)) // Only enters if there are agents that have not reach the goal
        {
			
			
			 auto begin = std::chrono::high_resolution_clock::now();
            
            
            if(visualizer)
            {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            }
            
             if(timestep==2)
                {
                    
                   // char u= getchar();
                }
            float nowtime=sim->getGlobalTime();
            for (size_t i = 0; i < sim->getNumAgents(); ++i)
            {
                //	srand(time(NULL));
                prevVel[i]=sim->getAgentVelocity(i);
                float x,y;
                
                needtoUpdate[i]=0;
                if(timestep==2)
                {
                    chosenAction[i]=0;
                   // char u= getchar();
                }
                
                if(i==agentViewed)
                {
                    //	std::cout << timestep <<" chosenAction is " << chosenAction[i] << " \n";
                    
                }
                
                
                if(timestep>1)
                {
                    
                    if((algo>=3)&&(!isinGoal[i]))
                    {
                        
                        
                        RVO::Vector2 lastPositionAgent= sim->getAgentPosition(i)+(-sim->getAgentVelocity(i)* sim->getTimeStep());
                        float goalP=((RVO::normalize(goals[i]-lastPositionAgent)* sim->getAgentVelocity(i))/(float)1.5);
                        if(i==0)
                        {
                           // std::cout << (RVO::normalize(goals[i]-lastPositionAgent)* sim->getAgentVelocity(i))<< " divided by 1.5 is " << goalP<< " \n";
                            
                        }
                        
                        
                        float energy=(1+1*(sim->getAgentVelocity(i)*sim->getAgentVelocity(i)))/(float)3.25;
                        float allowed=(sim->getAgentPrefVelocity(i)*sim->getAgentVelocity(i))/(float)2.25;//-(RVO::abs(sim->getAgentPrefVelocity(i)-sim->getAgentVelocity(i))-1.5)/(float)1.5;
                        
                        for(int tempAct=0;tempAct<Actions;tempAct++)
                        {
                            
                            Chosen[i][tempAct]=0;
                        }
                        
                                               
                     //   addTimeWindow2(i,chosenAction[i] ,  (((goalP*(1-coord_factor)+coord_factor*allowed))+1)/(float)2    );//RealReward[i][chosenAction[i]
                        addTimeWindow2(i,chosenAction[i] ,  (goalP*(1-coord_factor)+coord_factor*allowed)    );//RealReward[i][chosenAction[i]
                       
                       
                       if(i==agentViewed)
                        {
                            std::cout << timestep<< " chosenAction is " << chosenAction[i] <<  " reward:  " << goalP*(1-coord_factor)+coord_factor*allowed  << " epsilon " << epsilon[i]<< " \n";
                            
                        }
                       
                        
                        
                        
                    }
                    
                    
                    
                    if(((std::rand()/(float)RAND_MAX)<=updateProbability)||(timestep==2)) //If true, a new motion decision will be made, otherwise the agent keeps its previous action
                    {
                        setOptimization(i);
                        needtoUpdate[i]=1;
                       
                        if((std::rand()/(float)RAND_MAX)<=0.06667)
                       { 
						  // int randomaction=(rand()%Actions);
                         //  chosenAction[i]=randomaction;
						   
						   //chosenAction[i]=4;
						   }
                       else
                       {
						   
						  // chosenAction[i]=0;
						   
						   
						   }
						   
						   
       //  randomaction=(rand()%Actions);
      //  chosenAction[i]=randomaction;
                        
                    }
                    
                    
                }
                
                	if((timestep-1)%2==0) //This indicates how often the output to Pos_ori is produced
				{
					
					float alphaO = .9f;
					
					if (RVO::abs(sim->getAgentVelocity(i)) < .02)
					{
						alphaO = 1;
					}
					
					RVO::Vector2 normv ;
					
					normv = sim->getAgentVelocity(i);
					
					
					normalize(normv);
					
					RVO::Vector2 normpref = sim->getAgentPrefVelocity(i);
					
					normalize(normpref);
					
					float score = normv*normpref;
					
					if (score < 0) 
					{
						alphaO = 1;
					}
					else 
					{
						alphaO = .9;
					}
					float ordiff;
					
					
					ordiff = atan2(sim->getAgentVelocity(i).y(),sim->getAgentVelocity(i).x()) - ori[i];
					
					if (ordiff > M_PI)
					{
						ordiff-=2*M_PI;
					}
					
					if (ordiff < -M_PI)
					{
						ordiff+=2*M_PI;
					}
					
					ori[i]+= (1.f-alphaO)*ordiff;
					
					if (ori[i] > M_PI)
					{
						ori[i]-=2*M_PI;
					}
					
					if (ori[i] < -M_PI)
					{
						ori[i]+=2*M_PI;
					}
					
					//Writing the pos and orientation of the agent
					
				//	positionvel[i][0] = x;
					
					//positionvel[i][1] = y;

					//Output the agent information to a file PosOri
					// posangleagg << i<< ", " << 0 << ", "<< sim->getAgentPosition(i).x()  << ", " << sim->getAgentPosition(i).y() << ", " << cos(ori[i]) << ", " << sin(ori[i]) << ", 0.5, "<< nowtime << /*" vel " << sim->getAgentVelocity(i).y() <<*/"\n";
				
				}
                
                
                /***************Drawing the agent:*/
                
                if(visualizer)
                {
                    
                    glPushMatrix();
                    glTranslatef(sim->getAgentPosition(i).x(), sim->getAgentPosition(i).y(), 0.0f);
                    glColor3f(0.4f,0.9f,0.0f); //greenish color for the agents
                    if(i==0)
                    {
                        
                        glColor3f(0.9f,0.9f,0.9f); //greenish color for the agents
                    }
                    
                    glutSolidSphere(0.5f,8,8);
                    glDisable( GL_LIGHTING );
                    glColor3f(0,0,0);
                    
                    
                    glBegin(GL_LINES);
                    glVertex3f(0.f,0.f,1.0f);
                    RVO::Vector2 pfrev=  sim->getAgentPrefVelocity(i);
                    glVertex3f(pfrev.x(),pfrev.y(),1.0f);
                    glEnd();
                    glPopMatrix();
                    
                }
                
                
            } //End of the iteration of all agents
            
            
            
            
            
         //   auto begin = std::chrono::high_resolution_clock::now();

         // code to benchmark


                          
                            
                       

            
            if(algo==1) //ORCA
            {
                
                setPreferredVelocities();
                
            }
            
            if(algo==2) //C-Nav
            {
                
                if(timestep>1)
                {
                    
                    simulateVelocities();
                }
                setPreferredVelocities();
                
            }
            
            if((algo==3)||(algo==4)||(algo==5))
            {	
				if(timestep>4)
            {
				clock_t begin = clock();

  /* here, do your time-consuming job */
  UCB_angle();

  //clock_t end = clock();
  //double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    //std::cout << " Total time spent in calculating ACTION :" << time_spent << " agents not in goal: " <<  totalnotingoal<< "\n";
    //total_time_spent += time_spent;
    //sample_time_spent +=totalnotingoal;
                
                
                
            }
                setPreferredVelocities();
            }
            
           
            
            

           
   /* here, do your time-consuming job */
  sim->doStep(); //calls a method in RVOSimulator.cpp to update the simulation
   // char g = getchar();

          
         //   auto end = std::chrono::high_resolution_clock::now();
         // std::cout << " Total: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count()/(float)1000000000 << " s" << std::endl;

        // AvgTime=AvgTime + std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count() ;
        //totalcounts=totalcounts+1;
            //char p = getchar();
            countCollisions();
            
            for (size_t i = 0; i < sim->getNumAgents(); ++i)
            {
                //	srand(time(NULL));
                postVel[i]=sim->getAgentVelocity(i);
			}
            calcAccel(); 
           
            
            if(visualizer)
            {
                glPushMatrix();
                glTranslatef(0, 0, 0.0f);
                glColor3f(0.0f, 0.0f, 0.0f);
                
                
                if(scenario==1)
                {
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-1.5f,   -0.5f);
                    glVertex2f(-1.5f,   -1.5f);
                    glVertex2f(sidesize+1.5f, -1.5f);
                    glVertex2f(sidesize+1.5f, -0.5f);
                    glEnd();
                    
                    
                    glBegin(GL_QUADS);
                    glVertex2f(sidesize+0.5f,   sidesize+1.5f);
                    glVertex2f(sidesize+0.5f,   -1.5f);
                    glVertex2f(sidesize+1.5f, -1.5f);
                    glVertex2f(sidesize+1.5f, sidesize+1.5f);
                    glEnd();
                    
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-1.5f,   sidesize+1.5f);
                    glVertex2f(-1.5f,   sidesize+0.5f);
                    glVertex2f(sidesize+1.5f,  sidesize+0.5f);
                    glVertex2f(sidesize+1.5f,  sidesize+1.5f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-1.5f,   sidesize+1.5f);
                    glVertex2f(-1.5f,   -1.5f);
                    glVertex2f(-0.5f, -1.5f);
                    glVertex2f(-0.5f, sidesize+1.5f);
                    glEnd();
                    
                    /*
                    glBegin(GL_QUADS);
                    glVertex2f(((float)sidesize/(float)2)-3,   ((float)sidesize/(float)2)+3);
                    glVertex2f(((float)sidesize/(float)2)-3,   ((float)sidesize/(float)2)-3);
                    glVertex2f(((float)sidesize/(float)2)+3, ((float)sidesize/(float)2)-3);
                    glVertex2f(((float)sidesize/(float)2)+3,((float)sidesize/(float)2)+3);
                    glEnd();
                    */
                    
                }
                
                
                if(scenario==3)
                {
                    glBegin(GL_QUADS);
                    glVertex2f(200.0f,   31.0f);
                    glVertex2f(-200.0f,   31.0f);
                    glVertex2f(-200.0f, 30.5f);
                    glVertex2f(200.0f, 30.5f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(200.0f,   27.5f);
                    glVertex2f(-200.0f,   27.5f);
                    glVertex2f(-200.0f, 27.0f);
                    glVertex2f(200.0f, 27.0f);
                    glEnd();
                    
                    
                }
                
                if(scenario==4)
                {
               /*     glBegin(GL_QUADS);
                    glVertex2f(0.5f,   350.0f);
                    glVertex2f(-0.5f,   350.0f);
                    glVertex2f(-0.5f, 0.6f);
                    glVertex2f(0.5f, 0.6f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(0.5f,   -0.6f);
                    glVertex2f(-0.5f,   -0.6f);
                    glVertex2f(-0.5f, -150.0f);
                    glVertex2f(0.5f, -150.0f);
                    glEnd();*/
                    
                      glBegin(GL_QUADS);
                     glVertex2f(3.0f,200.0f);
                    glVertex2f(2.0f, 200.0f);
                    glVertex2f(2.0f, 3.0f);
                    glVertex2f(3.0f, 3.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-2.0f,   200.0f);
                    glVertex2f(-3.0f, 200.0f);
                    glVertex2f(-3.0f, 3.0f);
                    glVertex2f(-2.0f, 3.0f);
                    glEnd();
                      


                    glBegin(GL_QUADS);
                    glVertex2f(3.0f,   -3.0f);
                    glVertex2f(2.0f,   -3.0f);
                    glVertex2f(2.0f, -200.0f);
                    glVertex2f(3.0f, -200.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-2.0f, -3.0f);
                    glVertex2f(-3.0f,   -3.0f);
                    glVertex2f(-3.0f, -200.0f);
                    glVertex2f(-2.0f, -200.0f);
                    glEnd();


                                  
                    
                    
                      glBegin(GL_QUADS);
                     glVertex2f(200.0f,   3.0f);
                    glVertex2f(2.0f,   3.0f);
                    glVertex2f(2.0f, 2.0f);
                    glVertex2f(200.0f, 2.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(200.0f,   -2.0f);
                    glVertex2f(2.0f, -2.0f);
                    glVertex2f(2.0f, -3.0f);
                    glVertex2f(200.0f, -3.0f);
                    glEnd();
                      


                    glBegin(GL_QUADS);
                    glVertex2f(-2.0f,   3.0f);
                    glVertex2f(-200.0f,   3.0f);
                    glVertex2f(-200.0f, 2.0f);
                    glVertex2f(-2.0f, 2.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-2.0f, -2.0f);
                    glVertex2f(-200.0f,   -2.0f);
                    glVertex2f(-200.0f, -3.0f);
                    glVertex2f(-2.0f, -3.0f);
                    glEnd();

   
        
        
	  
                }
                
                
                if(scenario==5)
                {
                    glBegin(GL_QUADS);
                    glVertex2f(-10.0f,   350.0f);
                    glVertex2f(-11.0f,   350.0f);
                    glVertex2f(-11.0f, 0.6f);
                    glVertex2f(-10.0f, 0.6f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-10.0f,   -0.6f);
                    glVertex2f(-11.0f,   -0.6f);
                    glVertex2f(-11.0f, -150.0f);
                    glVertex2f(-10.0f, -150.0f);
                    glEnd();
                }
                
                
                /*if(scenario==6)
                {
				        
        
					glBegin(GL_QUADS);
                    glVertex2f(-4.1f,   350.0f);
                    glVertex2f(-5.1f,   350.0f);
                    glVertex2f(-5.1f,   -100.0f);
                    glVertex2f(-4.1f,   -100.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(-0.9f,   350.6f);
                    glVertex2f(-1.9f,   350.6f);
                    glVertex2f(-1.9f, -100.0f);
                    glVertex2f(-0.9f, -100.0f);
                    glEnd();
        	
					
				}
                */
                
                if(scenario==7)
                {
                    
                    glPushMatrix();
                    glTranslatef(0, 0, 0.0f);
                    glColor3f(0.0f, 0.0f, 0.0f);
                                                          
                    glBegin(GL_QUADS);
                    glVertex2f(  -10.0f,   2.0f);
                    glVertex2f(-10.0f,   0.6f);
                    glVertex2f(10.0f, 0.6f);
                    glVertex2f(  10.0f, 2.0f);
                    glEnd();
                    
                    glBegin(GL_QUADS);
                    glVertex2f(  -10.0f,   -0.6f);
                    glVertex2f(-10.0f,   -2.0f);
                    glVertex2f(10.0f, -2.0f);
                    glVertex2f(  10.0f, -0.6f);
                    glEnd();
                    
                    
                }
                
                
                if(scenario==8)
                {	
					
				
        
         glBegin(GL_QUADS);
                    glVertex2f(-2.0f,   2.0f);
                    glVertex2f(0.0f,   -2.0f);
                    glVertex2f(2.0f, -2.0f);
                    glVertex2f(2.0f, 2.0f);
                    glEnd();
					
					
        /*
         glBegin(GL_QUADS);
                    glVertex2f(-1.0f,   1.0f);
                    glVertex2f(-1.0f,   -1.0f);
                    glVertex2f(1.0f, -1.0f);
                    glVertex2f(1.0f, 1.0f);
                    glEnd();
					*/
						/* 
                    glBegin(GL_QUADS);
                    glVertex2f(-1.0f,   1.0f);
                    glVertex2f(-1.0f,   -1.0f);
                    glVertex2f(1.0f, -1.0f);
                    glVertex2f(1.0f, 1.0f);
                    glEnd();
					
						      
                    glBegin(GL_QUADS);
                    glVertex2f(-1.0f,   0.5f);
                    glVertex2f(-1.0f,   -1.5f);
                    glVertex2f(1.0f, -1.5f);
                    glVertex2f(1.0f, 0.5f);
                    glEnd();
                    
                      glBegin(GL_QUADS);
                    glVertex2f(3.0f,   3.5f);
                    glVertex2f(3.0f,   1.5f);
                    glVertex2f(5.0f, 1.5f);
                    glVertex2f(5.0f, 3.5f);
                    glEnd();
                    
                      glBegin(GL_QUADS);
                    glVertex2f(-4.5f,   -2.0f);
                    glVertex2f(-4.5f,   -4.0f);
                    glVertex2f(-2.5f, -4.0f);
                    glVertex2f(-2.5f, -2.0f);
                    glEnd();
                    
                       glBegin(GL_QUADS);
                    glVertex2f(-1.5f,   6.5f);
                    glVertex2f(-1.5f,   4.5f);
                    glVertex2f(0.5f, 4.5f);
                    glVertex2f(0.5f, 6.5f);
                    glEnd();
                    
                    
                      glBegin(GL_QUADS);
                    glVertex2f(3.0f,   0.0f);
                    glVertex2f(3.0f,   -2.0f);
                    glVertex2f(5.0f, -2.0f);
                    glVertex2f(5.0f, 0.0f);
                    glEnd();
                    
                      glBegin(GL_QUADS);
                    glVertex2f(7.0f,   3.5f);
                    glVertex2f(7.0f,   1.5f);
                    glVertex2f(9.0f, 1.5f);
                    glVertex2f(9.0f, 3.5f);
                    glEnd();
                    
                      glBegin(GL_QUADS);
                    glVertex2f(-0.5f,   -3.0f);
                    glVertex2f(-0.5f,   -5.0f);
                    glVertex2f(1.5f, -5.0f);
                    glVertex2f(1.5f, -3.0f);
                    glEnd();
                    
                       glBegin(GL_QUADS);
                    glVertex2f(2.5f,   7.0f);
                    glVertex2f(2.5f,   5.0f);
                    glVertex2f(4.5f, 5.0f);
                    glVertex2f(4.5f, 7.0f);
                    glEnd();
                    
          
    */
        
					
				}
                glPopMatrix();
                
            }
            
            
            
  auto end = std::chrono::high_resolution_clock::now();
    //std::cout << " Total: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count()/(float)1000000000 << " s" << std::endl;

     AvgTime=AvgTime + std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count() ;
    totalcounts=totalcounts+1;
            
            
            
            
        }
        else //If all agents have reached their goals
        {
            //  finalTime=sim->getGlobalTime();
            finalize=1;
            posangleagg << "***********************************************************************************\n"; //Separete between agents pos and vel in different iterations
			
            
            if(timestep>=threshold)
            {
				
			std::cout << "TIME's UPPP!!!\n";	
			}
            
            float totalAccel=0, totalTimeToGoal=0;
            
            for(int i=0;i<Agents;i++)
            {
				if(isinGoal[i])
				{
                totalTimeToGoal+=TimetoGoal[i];
				//std::cout << "Agent " << i << " made it " <<TimetoGoal[i] <<  "\n";
                globalAvgReward= globalAvgReward + cumReward[i];//(float)TimetoGoal[i];
                globalGoalOriented = globalGoalOriented + totalGoalOriented[i]/(float)TimetoGoal[i];
                //std::cout << "Agent " << i << " in pos: " <<sim->getAgentPosition(i) << " and Goal: " << goals[i] <<" made it " <<TimetoGoal[i] <<  " and globalGoalOriented: " <<globalGoalOriented <<" \n";
                }
				else
				{
				//	std::cout << "Agent " << i << " didnt make it " << sim->getGlobalTime() <<  "\n";
					TimetoGoal[i]= sim->getGlobalTime();
				totalTimeToGoal+=	TimetoGoal[i];
				}
            }
            //getchar();
            totalrewardsimulation=totalrewardsimulation+totalTimeToGoal/(float)Agents;
            
         
	//	std::cout << " AvgRw: " << totalTimeToGoal/(float)Agents <<  "\n";
            
            float AvgStd=0, tempstd=0;
            AvgStd=AvgStd+ totalrewardsimulation;
            
            for(int i=0;i<Agents;i++)
            {
                tempstd=tempstd+ ((TimetoGoal[i]-totalrewardsimulation)*(TimetoGoal[i]-totalrewardsimulation));
				//std::cout << " Stdev: " << tempstd <<  " from " << TimetoGoal[i] << " and " << totalrewardsimulation<<"\n";	
			
            }
            
            if(Agents>1)
            {
            tempstd=tempstd/(float)(Agents-1);
			}
			else
			{
			 tempstd=0;
			
			}
            tempstd=sqrt(tempstd);
			//std::cout << " Stdev: " << tempstd <<"\n";	
            AvgStd=AvgStd+3*tempstd;
            finalTime=AvgStd;
          // std::cout << " Score: " <<finalTime-baseScore+0.5<< "\n";
            SimScore=SimScore+finalTime;
            
        }
        //Closing File Buffers
	posangleagg.close();
    }
    
    
    
    if(visualizer)
    {
        glutSwapBuffers();
        if(finalize)
        {
           glutLeaveMainLoop();
            
        }
    }
    
}

//This commented function displays the x,y positions of the agents
/*
 void showPositions()
 {
	// Output the current global time.
	std::cout << sim->getGlobalTime();
 
	// Output the current position of all the agents.
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
 std::cout << " " << sim->getAgentPosition(i);
	}
 
	std::cout << std::endl;
 }
 */

//This function sets up the actions and the RVO parameters for each iteration of the experiment
float crowd_simulation_eval(const float* actionsDir, const float* actionsMag, int num_actions)
{
    Actions = num_actions;
    for (int i = 0; i < Actions; i++)
    {
        actionVector[i] = actionsDir[i];
        actionVectorMag[i] = actionsMag[i];
    }
	
	
    
    sim = new RVO::RVOSimulator();
    iteration=1;
    setupScenario(sim);
    
    if(visualizer)
    {
        glutMainLoop();
    }
    else
    {
        do {
            updateVisualization();
        }
        while (!finalize);
    }
    delete sim;
    
    return finalTime;
    
}

int main(int argc, char* argv[])
{
    
    if( argc == 7 )
    {
          visualizer= atoi(argv[3]);
        algo = atoi(argv[1]);
        finalIteration= atoi(argv[4]);
        coord_factor= atof(argv[5]);
        sizeTW=atoi(argv[6]);
        agentViewed=999;//atoi(argv[7]);
        
        
    }
    else if( argc > 7 )
    {
        std::cout <<"Too many arguments supplied.\n";
        exit(1);
    }
    else
    {
        std::cout << "Indicate the algorithm, scenario, visualization option and number of iterations.\n Algorithm:\n 1.- ORCA\n 2.- C-Nav\n 3.- ALAN\n 4.- WUCB\n 5.- e-greedy\n  \n Scenarios:\n 1.- Crowd\n 2.- Circle\n 3.- Bidirectional\n 4.- PerpCrossing\n 5.- Congested\n 6.- Crossing\n 7.- Deadlock\nVisualization:\n 0.-Off\n 1.-On\n \n Number of iterations\n\n Coordination Factor\n    ";
        exit(1);
    }
    
    
    
     srand(time(NULL));
    
   /*  int sce=6;
     visualizer= 0;//atoi(argv[3]);
     algo = 3;//atoi(argv[1]);
     finalIteration= 10;//atoi(argv[4]);
     coord_factor= 0.6;//atof(argv[5]);
     sizeTW=40;//atoi(argv[6]);
     agentViewed=999;//atoi(argv[7]);
*/
    
    ////////////////////////////////////////////////////////////////////
    //////Sample actions: 4 velocities whose direction is in radians (velDir) and the magnitudes are in meters per second (velMag)
  
  
float velDir[6] = {0.00000,1.445935,4.250190, 6.245753, 3.766477, 3.944874}; //Learned Action Set directions
 
   
//float velDir[8] = {0.00000,-M_PI/4, M_PI/4, M_PI/2,M_PI+M_PI/4,M_PI, M_PI-M_PI/4,-M_PI/2 };
	
	
	float velMag[6] = {1.5,  1.5,1.5, 1.5 ,1.5, 1.5}; // Action set magnitudes

 //float velMag[8] = {1.5,  1.5,1.5, 1.5,1.5,  1.5,1.5, 1.5 };
  decisionSteps=0;
    ///////////////////////////////////////////////////////////////////
    float Result[400];

      for(int i=0;i<Agents;i++)
      {
		for (int u=0; u<8; u++)
		{
	
        count[i][u]=0;
         }
    
      }
      
    for(int i=0;i<finalIteration; i++) //The code below is executed for each iteration
    {
        if(visualizer)
        {
            glutInit(&argc, argv);
            glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
            glutInitWindowSize(800, 600);
            glutCreateWindow("ORCA");
            glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
            InitGL();
            glutDisplayFunc(updateVisualization);
            glutReshapeFunc(reshape);
            glutIdleFunc(idle);
        }
        
        finalize=0;
        
        
        switch(atoi(argv[2]))
        {
            case 1:Agents=300,scenario=1;
                break;
                
            case 2:Agents=80,scenario=2;
                break;
                
            case 3:Agents=18,scenario=3;
                break;
                
            case 4:Agents=80,scenario=4;
                break;
                
            case 5:Agents=32,scenario=5;
                break;
                
            case 6:Agents=16,scenario=6;
                break;
                
            case 7:Agents=10,scenario=7;
                break;
            case 8:Agents=5,scenario=8;
                break;
        }
        
      clock_t begin = clock();
        Result[i]=  crowd_simulation_eval(velDir, velMag, (int)(sizeof(velDir)/sizeof(float)));
        clock_t end = clock();
         double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    //std::cout << " Total time spent in calculating ACTION :" << time_spent << " agents not in goal: " <<  totalnotingoal<< "\n";
    total_time_spent += time_spent;
    sample_time_spent +=1;
        //std::cout << Result[i]-baseScore+0.5<<  "\n"; // this method takes as input information about the actions/velocities: the vector of directions, the vector of magnitudes and the number of actions
        
    }
    
  
    float ResultStd=0;
    for(int i=0;i<finalIteration;i++)
				{
                    ResultStd=ResultStd+ ((Result[i]-(SimScore/(float)finalIteration) )*(Result[i]-(SimScore/(float)finalIteration) ));
                    
                }
				
				ResultStd=ResultStd/(float)(finalIteration-1);
				ResultStd=sqrt(ResultStd);
				ResultStd=(ResultStd)/(float)sqrt(finalIteration);
    
   std::cout <<  (SimScore/(float)finalIteration) -baseScore+0.5 << ", " << ResultStd<< ", " << baseScore << ", Col " << colisiones<< " AvgTime " << (AvgTime/(float)totalcounts)/(float)1000000000<<" ns " << (float)totnumcan/(float)samp<< "\n";
   //std::cout << "Global Avg reward: " << globalAvgReward/(float)decisionSteps << " GlobalGoalOriented= " << globalGoalOriented <<"\n";
   std::cout << "TIME PER ACTION: "<<  total_time_spent/sample_time_spent<< "\n";
   std::cout << "AVERAGE ACTION CHANGES " << (float)action_change/(float)action_step << "\n";
   int totalCount=0; //", accel: " << cumAccel/(float)finalIteration << ", " <<maxAccel << "\n";//
   for (int u=0;u<(int)(sizeof(velDir)/sizeof(float));u++)
   {
	  
		 // std::cout << " Count " << u <<" : " << count[0][u] <<"\n";
		 totalCount=totalCount+count[0][u];
	   
	  }
	  
	   for (int u=0;u<(int)(sizeof(velDir)/sizeof(float));u++)
   {
	  
		// std::cout << " Count " << u <<" : " << (float)count[0][u]/(float)totalCount <<"\n";
		 
	   
	  }
   
    //std::cout << " Final Score: " << SimScore/(float)finalIteration << " CoordFactor: " << coord_factor<<"\n";
    
  // std::cout << " Collisiones: "  << (float)colisiones/(float)finalIteration <<"\n";
    return 0;
    
}

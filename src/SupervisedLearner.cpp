#include <fstream>
#include <iostream>

#include <ros/package.h>

#include "SupervisedLearner.h"

SupervisedLearner::SupervisedLearner()
{
	// Input file
	ifstream slMatFile(ros::package::getPath("rl_nav")+"/slMatData.txt");

	// Check for failure
	if(!slMatFile.is_open())
	{
		cout<<"Invalid sl file"<<endl;
		slValid = false;
	}
	else 
	{
		slValid = true;
		// Variables for reading values		
		int stateDir;
		int stateHead;
		int stateFOV;
		int slPrediction;
		float slValue;

		for(int i = 0; i<STATE_DIR_MAX; i++)
			for(int j = 0; j<STATE_HEAD_MAX; j++)
				for(int k = 0; k<STATE_FOV_MAX; k++)
					{
						slMatFile >> stateDir >> stateHead >> stateFOV >> slPrediction >> slValue;
						slPredictionMatrix[stateDir][stateHead][stateFOV] = bool(slPrediction);
						slValueMatrix[stateDir][stateHead][stateFOV] = slValue;
					}
	}

	// Close
	slMatFile.close();
}

// Function to return SL prediction
bool SupervisedLearner::predict(vector<int> stateAction)
{
	if(!slValid)
		return false;

	int stateDir = stateAction[0];
	int stateHead = stateAction[1];
	int stateFOV = stateAction[2];

	// Get SL prediction
	return slPredictionMatrix[stateDir][stateHead][stateFOV];
}

// Function to return SL value
float SupervisedLearner::distance(vector<int> stateAction)
{
	if(!slValid)
		return 0.0;

	int stateDir = stateAction[0];
	int stateHead = stateAction[1];
	int stateFOV = stateAction[2];

	// Get SL value
	return slValueMatrix[stateDir][stateHead][stateFOV];
}

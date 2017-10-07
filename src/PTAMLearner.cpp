#include <fstream>

#include "Helper.h"
#include "PTAMLearner.h"

#include <ros/package.h>

using namespace std;

pthread_mutex_t PTAMLearner::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t PTAMLearner::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
int PTAMLearner::MAX_POINT_OVERLAP;

PTAMLearner::PTAMLearner()
{
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &PTAMLearner::pointCloudCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &PTAMLearner::gazeboModelStatesCb, this);
	srand (time(NULL));
	ros::NodeHandle p_nh("~");
	p_nh.getParam("MAX_POINT_OVERLAP", MAX_POINT_OVERLAP);
	lastBestQStateAction = nullTuple;
}

void PTAMLearner::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);	
	robotWorldPose = modelStatesPtr->pose.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

void PTAMLearner::pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr)	
{
	pthread_mutex_lock(&pointCloud_mutex);
	currentPointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

//convert all the possible trajectories into state-action format and store them
void PTAMLearner::getActions()
{
	if(!possibleTrajectories.size())
		for(auto trajectory : Helper::getPoses())
			possibleTrajectories.push_back(getAction(trajectory));
}

//convert a given bernstein input relative to the robot camera frame into a state-action pair and get the Q value
CommandStateActionQ PTAMLearner::getAction(geometry_msgs::PoseStamped inputPose)
{
	vector<int> rl_input;
	pcl::PointCloud<pcl::PointXYZ> nextPointCloud = Helper::getPCLPointCloudAtPosition(inputPose);
	
	pthread_mutex_lock(&pointCloud_mutex);
	vector<pcl::PointXYZ> commonPoints = Helper::pointCloudIntersection(currentPointCloud,nextPointCloud);
	pthread_mutex_unlock(&pointCloud_mutex);

	float dir = Helper::sign(inputPose.pose.position.x), del_heading = Helper::Quat2RPY(inputPose.pose.orientation)[2];
	//RL params
	rl_input.push_back((int)(dir==1)?1:0);
	if(fabs(del_heading)*180.0/PI > 30)
		rl_input.push_back(19);
	else
		rl_input.push_back((int) 19*fabs((del_heading)*180.0/(30*PI)));

	rl_input.push_back((int) min(19,commonPoints.size()*20.0/((float)MAX_POINT_OVERLAP)));

	return make_tuple(inputPose,rl_input,getQ(rl_input));
}

//returns the state-action with the highest Q value except the previously executed command so that it doesn't undo it
CommandStateActionQ PTAMLearner::getBestQStateAction(geometry_msgs::PoseStamped lastPose)
{
	cout<<"getBestQStateAction start"<<endl;
	if(get<1>(lastBestQStateAction)!=get<1>(nullTuple) and get<2>(lastBestQStateAction)!=get<2>(nullTuple))
		return lastBestQStateAction;
	cout<<"lastBestQStateAction not null"<<endl;
	vector<CommandStateActionQ> result;
	int index=-1;
	float maxQ = -numeric_limits<float>::infinity();  //init max Q to negative infinity
	float Q;
	cout<<"getting actions"<<endl;
	getActions();
	cout<<"evaluating actions"<<endl;
	for(auto input : possibleTrajectories)
	{	
		geometry_msgs::PoseStamped inp = get<0>(input);
		if(!(Helper::sign(lastPose.pose.position.x) + Helper::sign(inp.pose.position.x)) and 
			!(Helper::Quat2RPY(lastPose.pose.orientation)[2] + Helper::Quat2RPY(inp.pose.orientation)[2]) and 
			!(fabs(Helper::Quat2RPY(lastPose.pose.orientation)[2]) - fabs(Helper::Quat2RPY(inp.pose.orientation)[2])))
			continue;
		result.push_back(input);
		Q = get<2>(input);
		if(maxQ<Q)
		{
			maxQ = Q;
			index = result.size()-1;
		}
	}
	cout<<"finished evaluating actions"<<endl;
	if(maxQ == -numeric_limits<float>::infinity() or !result.size())
	{
		cout<<"result size zero"<<endl;
		if(possibleTrajectories.size())
		{
			cout<<"returning random from possibleTrajectories of size "<<possibleTrajectories.size()<<endl;
			lastBestQStateAction = possibleTrajectories[rand()%possibleTrajectories.size()];
		}
		else
		{
			cout<<"returning random"<<endl;
			lastBestQStateAction = nullTuple;
			return getRandomStateAction();
		}
	}	

		
	else if(index!=-1 and result.size())
	{
		cout<<"setting lastBestQStateAction"<<endl;
		lastBestQStateAction = result[index];
	}
	cout<<"getBestQStateAction end"<<endl;
	return lastBestQStateAction;
}

//epsilon greedy policy
CommandStateActionQ PTAMLearner::getEpsilonGreedyStateAction(float epsilon, geometry_msgs::PoseStamped lastPose)
{
	if((rand() % 100) < epsilon)
		return getBestQStateAction(lastPose);
	else
		return getRandomStateAction();
}

//random policy
CommandStateActionQ PTAMLearner::getRandomStateAction()
{
	vector<geometry_msgs::PoseStamped > trajectories = Helper::getPoses();	
	cout<<trajectories.size()<<endl;
	return getAction(trajectories[rand()%trajectories.size()]);
}

//thresholded random, i.e. returns an action with a Q value higher than qThreshold
CommandStateActionQ PTAMLearner::getThresholdedRandomStateAction(float qThreshold, int maxIters)
{
	CommandStateActionQ result;

	do 
	{
		result = getRandomStateAction();
		maxIters--;
	} while(get<2>(result) < qThreshold and maxIters>=0);

	return result;
}

//Thresholded aligning with global path
CommandStateActionQ PTAMLearner::getThresholdedClosestAngleStateAction(float qThreshold, float nextAngle, geometry_msgs::PoseStamped lastPose)
{
	vector<CommandStateActionQ> potentialInputs;
	getActions();
	
	for(auto input : possibleTrajectories)
		if(get<2>(input) > qThreshold)
			potentialInputs.push_back(input);
	  
	if(potentialInputs.size()<1)
		return getBestQStateAction(lastPose);
	else
	{
		cout<<"THRESHOLDED SIZE "<<potentialInputs.size()<<endl;
		CommandStateActionQ result = potentialInputs[rand()%potentialInputs.size()];
		//need to change this to take the SLAM pose
		float currentAngle = Helper::Quat2RPY(robotWorldPose.orientation)[2], min_diff = numeric_limits<float>::infinity(), angle_diff;
		
		for(auto input : potentialInputs)
		{	
			geometry_msgs::PoseStamped command = get<0>(input);
			angle_diff = fabs(nextAngle - (currentAngle + Helper::Quat2RPY(command.pose.orientation)[2]));
			if(angle_diff<=min_diff)
			{
				min_diff = angle_diff;
				result = input;
			}
		}
		return result;
	}
}

//get supervised learner actions
vector<CommandStateActionQ> PTAMLearner::getSLActions()
{
	if(!slValid)
		return vector<CommandStateActionQ>();

	vector<CommandStateActionQ> potentialInputs;
	getActions();
	for(auto input : possibleTrajectories)
		if(!predict(get<1>(input)))
			potentialInputs.push_back(input);
	return potentialInputs;
}	

////get supervised learner actions with angle closest to nextAngle
CommandStateActionQ PTAMLearner::getSLClosestAngleStateAction(float nextAngle)
{
	if(!slValid)
		return getRandomStateAction();

	vector<CommandStateActionQ> potentialInputs = getSLActions();
	float currentAngle = Helper::Quat2RPY(robotWorldPose.orientation)[2], min_diff = numeric_limits<float>::infinity(), angle_diff;
		
	CommandStateActionQ result = getRandomStateAction();
	for(auto input : potentialInputs)
	{
		geometry_msgs::PoseStamped command = get<0>(input); 
		angle_diff = fabs(nextAngle - (currentAngle + Helper::Quat2RPY(command.pose.orientation)[2]));
		if(angle_diff<=min_diff)
		{
			min_diff = angle_diff;
			result = input;
		}
	}
	return result;
}

CommandStateActionQ PTAMLearner::getSLRandomStateAction()
{
	if(!slValid)
		return getRandomStateAction();

	vector<CommandStateActionQ> potentialInputs = getSLActions();
	
	if(potentialInputs.size())
		return potentialInputs[rand()%potentialInputs.size()];
	else
		return getRandomStateAction();
}

CommandStateActionQ PTAMLearner::getBestSLStateAction(geometry_msgs::PoseStamped lastPose)
{
	if(!slValid)
		return getBestQStateAction(lastPose);

	vector<CommandStateActionQ> result;
	vector<CommandStateActionQ> potentialInputs = getSLActions();
	int index;
	float maxDist = -numeric_limits<float>::infinity();  //init max Distance to -infinity
	float dist;
	
	for(auto input : potentialInputs)
	{	
		geometry_msgs::PoseStamped inp = get<0>(input);
		if(!(Helper::sign(lastPose.pose.position.x) + Helper::sign(inp.pose.position.x)) and 
			!(Helper::Quat2RPY(lastPose.pose.orientation)[2] + Helper::Quat2RPY(inp.pose.orientation)[2]) and 
			!(fabs(Helper::Quat2RPY(lastPose.pose.orientation)[2]) - fabs(Helper::Quat2RPY(inp.pose.orientation)[2])))
			continue;
		result.push_back(input);
		dist = distance(get<1>(result.back()));	
		if(maxDist<dist)
		{
			maxDist = dist;
			index = result.size()-1;
		}
	}

	if(maxDist == -numeric_limits<float>::infinity())
		return getSLRandomStateAction();
	
	return result[index];
}

//lastBestQStateAction is stored so that it doesn't need to be recomputed each time
void PTAMLearner::clear()
{
	possibleTrajectories.clear();
	lastBestQStateAction = nullTuple;
}
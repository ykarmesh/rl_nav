#include <vector>
#include <iostream>
#include <utility>
#include <tuple>
#include <cmath>

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>

#include "SarsaLearner.h"
#include "SupervisedLearner.h"

using namespace std;

typedef tuple<geometry_msgs::PoseStamped, vector<int>, float> CommandStateActionQ;
static CommandStateActionQ nullTuple = make_tuple(geometry_msgs::PoseStamped(), vector<int>(), -numeric_limits<float>::infinity());

class PTAMLearner : public SarsaLearner, public SupervisedLearner
{
private:
	ros::NodeHandle nh;
	ros::Subscriber pose_sub, pointCloud_sub;
	static pthread_mutex_t pose_mutex, pointCloud_mutex;
	static int MAX_POINT_OVERLAP;

	geometry_msgs::PoseStamped pose;
	pcl::PointCloud<pcl::PointXYZ> currentPointCloud;
	vector<CommandStateActionQ> possibleTrajectories;
	CommandStateActionQ lastBestQStateAction;
	void poseCb(const geometry_msgs::PoseStampedPtr posePtr);
	void pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr);	

public:	
	PTAMLearner();

	CommandStateActionQ getAction(geometry_msgs::PoseStamped inputPose);
	CommandStateActionQ getAction(geometry_msgs::PoseStamped inputPose, pcl::PointCloud<pcl::PointXYZ> nextPointCloud);
	CommandStateActionQ getRandomStateAction();
	CommandStateActionQ getThresholdedRandomStateAction(float qThreshold, int maxIters);
	CommandStateActionQ getBestQStateAction(geometry_msgs::PoseStamped lastPose);
	CommandStateActionQ getEpsilonGreedyStateAction(float epsilon, geometry_msgs::PoseStamped lastPose);
	CommandStateActionQ getThresholdedClosestAngleStateAction(float qThreshold, float nextAngle, geometry_msgs::PoseStamped lastPose);
	CommandStateActionQ getSLClosestAngleStateAction(float nextAngle);
	CommandStateActionQ getSLRandomStateAction();
	CommandStateActionQ getBestSLStateAction(geometry_msgs::PoseStamped lastPose);
	vector<CommandStateActionQ> getSLActions();
	void clear();
	void getActions();
};	

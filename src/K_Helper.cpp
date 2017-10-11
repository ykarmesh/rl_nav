#include <iostream>
#include <fstream>
#include "Helper.h"
#include <geometry_msgs/PoseArray.h>

using namespace std;

pthread_mutex_t Helper::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::costmap_mutex = PTHREAD_MUTEX_INITIALIZER;

geometry_msgs::PoseStamped Helper::pose;
//ptam_com::ptam_info Helper::ptamInfo;
std_msgs::Bool Helper::ptamInfo;
geometry_msgs::Pose Helper::robotWorldPose;
pcl::PointCloud<pcl::PointXYZ> Helper::currentPointCloud;
ros::ServiceClient Helper::posePointCloudClient;
int Helper::MAP;
bool Helper::up, Helper::down, Helper::left, Helper::right;
ros::Publisher Helper::next_poses_pub;
nav_msgs::OccupancyGrid Helper::grid;
Helper::Helper()
{
	next_poses_pub = nh.advertise<geometry_msgs::PoseArray>("/my_next_poses",1);
	posePointCloudClient = nh.serviceClient<ORB_SLAM2::PosePointCloud>("/ORB_SLAM2/posepointcloud");
	pose_sub = nh.subscribe("/vslam/pose_world",100, &Helper::poseCb, this);
	info_sub = nh.subscribe("/vslam/info",100, &Helper::ptamInfoCb, this);
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &Helper::pointCloudCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &Helper::gazeboModelStatesCb, this);
	OccupancyGrid_sub = nh.subscribe("/move_base/global_costmap/costmap", 1, &Helper::OccupancyGridCb, this);
	MAP=-1;
	up = down = left = right = true;
	ros::NodeHandle p_nh("~");
	p_nh.getParam("map", MAP);
}

//CALLBACKS---------------------------------------------------------------
void Helper::poseCb(const geometry_msgs::PoseStampedPtr posePtr)
{
	pthread_mutex_lock(&pose_mutex);
	pose = *posePtr;
	pthread_mutex_unlock(&pose_mutex);
}

//void Helper::ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr)
void Helper::ptamInfoCb(const std_msgs::BoolPtr ptamInfoPtr)
{
	pthread_mutex_lock(&info_mutex);
	ptamInfo = *ptamInfoPtr;
	pthread_mutex_unlock(&info_mutex);
}

void Helper::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);
	robotWorldPose = modelStatesPtr->pose.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}

void Helper::pointCloudCb(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr)
{
	pthread_mutex_lock(&pointCloud_mutex);
	currentPointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

void Helper::OccupancyGridCb(const nav_msgs::OccupancyGrid::ConstPtr &OGPtr)
{
	pthread_mutex_lock(&costmap_mutex);
	grid = *OGPtr;
	pthread_mutex_unlock(&costmap_mutex);
}
//-----------------------------------------------------------------------

//Get ROS pointcloud2 at position relative to camera frame given as bernstein input
sensor_msgs::PointCloud2 Helper::getPointCloud2AtPosition(vector<float> input)
{
	ORB_SLAM2::PosePointCloud posePointCloud;

	//PoseStamped from the new point
	pthread_mutex_lock(&pose_mutex);
	posePointCloud.request.pose = getPoseFromInput(input, Helper::pose);
	pthread_mutex_unlock(&pose_mutex);

	posePointCloudClient.call(posePointCloud);
	return posePointCloud.response.pointCloud;
}

//Get PCL pointcloud at position relative to camera frame given as bernstein input
pcl::PointCloud<pcl::PointXYZ> Helper::getPCLPointCloudAtPosition(vector<float> input)
{
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	pcl::fromROSMsg(Helper::getPointCloud2AtPosition(input), pointCloud);
	return pointCloud;
}

//Quaternion to RPY
vector<double> Helper::getPoseOrientation(geometry_msgs::Quaternion quat)
{
	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(quat, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return {roll, pitch, yaw};
}

//CHANGE or REMOVE
//Convert bernstein input in camera frame to pose in world frame
geometry_msgs::PoseStamped Helper::getPoseFromInput(vector<float> input, geometry_msgs::PoseStamped pose)
{
	geometry_msgs::PoseStamped p_out;
	geometry_msgs::Pose currentPose, p, newPose;
	tf::Quaternion currentQuat;
	tf::Pose currentTfPose, newTfPose;

	currentPose = pose.pose;

	p.position.z = input[3];
	p.position.x = -input[4]*input[12];
	p.position.y = 0.0;
	p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -input[12]*atan(input[5]),0.0);
	tf::quaternionMsgToTF(currentPose.orientation, currentQuat);
	tf::Transform currentTF(tf::Matrix3x3(currentQuat), tf::Vector3(currentPose.position.x,currentPose.position.y,currentPose.position.z));

	tf::poseMsgToTF(p, currentTfPose);
	newTfPose = currentTF * currentTfPose;
	tf::poseTFToMsg(newTfPose, newPose);

	p_out.header = pose.header;
	p_out.header.frame_id = "world";
	p_out.pose = newPose;
	return p_out;
}

//intersection of 2 pointclouds
vector<pcl::PointXYZ> Helper::pointCloudIntersection(pcl::PointCloud<pcl::PointXYZ> pointCloudA, pcl::PointCloud<pcl::PointXYZ> pointCloudB)
{
	vector<pcl::PointXYZ> commonPoints(pointCloudB.width * pointCloudB.height + pointCloudA.width * pointCloudA.height);
	vector<pcl::PointXYZ>::iterator it;
	it=set_intersection(pointCloudB.points.begin(), pointCloudB.points.end(),
						pointCloudA.points.begin(), pointCloudA.points.end(), commonPoints.begin(),pointEqComparer());
	commonPoints.resize(it-commonPoints.begin());
	return commonPoints;
}

//May require changes
//for collision avoidance, can be replaced with any collision avoidance algorithm
bool Helper::inLimits(geometry_msgs::PointStamped point)
{
	geometry_msgs::PointStamped point_odom;
	try
	{
		listener.transformPoint("/odom", point, point_odom);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s",ex.what());
		return;
	}
	float res = grid.info.resolution;
	int cellx = (int)point_odom.point.x/res;
	int celly = (int)point_odom.point.y/res;
  float data = grid.data[cellx+(celly-1)*grid.info.width];
	if((data < 50)&&(data!=-1))
	{
		cout<<"waypoint is in free space";
		return true;
	}
	cout<<"waypoint is failing";
	return false;
}

//CHANGE
//generate the recovery actions as bernstein inputs
vector<vector<float> > Helper::getTrajectories()
{
	float angle = PI/90.0, num_angles = 14;//, x, y;
	geometry_msgs::PointStamped point;
	vector<vector<float> > inputs;
	vector<double> orientation = getPoseOrientation(robotWorldPose.orientation);
	geometry_msgs::PoseArray poseArray;
	poseArray.header = Helper::pose.header;
	poseArray.header.frame_id = "world";
	for(float i=-num_angles*angle ; i<=num_angles*angle ; i+=angle)
	{
		vector<float> inp = {0.0,0.0,0.0,
								 cos(i), sin(i), tan(i),
								 0.0,0.0,0.0,0.0,0.0,0.0,
								 1.0,0.0,1.5};
		if(up)
		{
			if(i<0 and !right)
				continue;
			if(i>0 and !left)
				continue;
			x = robotWorldPose.position.x + cos(orientation[2] + i);
			y = robotWorldPose.position.y + sin(orientation[2] + i);
			if(inLimits(x,y))// and collisionFree(robotWorldPose.position.x, x, robotWorldPose.position.y, y, i, 1, orientation[2]))
			{
				poseArray.poses.push_back(getPoseFromInput(inp, Helper::pose).pose);
				inputs.push_back(inp);
			}
		}

		if(down)
		{
			if(i>0 and !left)
				continue;
			if(i<0 and !right)
				continue;
			inp[3] *= -1.0;
			inp[4] *= -1.0;
			//inp[5] *= -1.0;
			inp[12] *= -1.0;
			x = robotWorldPose.position.x - cos(orientation[2] - i);
			y = robotWorldPose.position.y - sin(orientation[2] - i);
			if(inLimits(x,y))// and collisionFree(robotWorldPose.position.x, x, robotWorldPose.position.y, y, -i, -1, orientation[2]))
			{
				poseArray.poses.push_back(getPoseFromInput(inp, Helper::pose).pose);
				inputs.push_back(inp);
			}
		}
	}
	next_poses_pub.publish(poseArray);
	return inputs;
}

//save episode to file
void Helper::saveFeatureExpectation(vector<vector<vector<int> > > episodeList, string fileName)
{
	ofstream feFile(fileName);
	for(auto episode : episodeList)
		for(auto rlStep : episode)
		{
			for(auto i : rlStep)
				feFile<< i << '\t';
			feFile << endl;
		}
}

//read episode from file
vector<vector<vector<int> > > Helper::readFeatureExpectation(string fileName)
{
	vector<vector<vector<int> > > episodeList = vector<vector<vector<int> > >();
	vector<vector<int> > episode = vector<vector<int> >();
	ifstream infile(fileName);
	int num_episodes = 0;
	if(infile.good())
	{
		int dir, angle, fov, status;
		while(infile)
		{
			infile >> dir >> angle >> fov >> status;
			episode.push_back({dir, angle, fov, status});

			if(status==1)
			{

				if(!episodeList.size() or (episodeList.size() and episode != episodeList.back()))
				{
					episodeList.push_back(episode);
					num_episodes++;
				}
				episode.clear();
			}
		}
	}

	return episodeList;
}

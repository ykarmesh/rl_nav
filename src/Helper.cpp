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
bool Helper::up, Helper::down, Helper::left, Helper::right;
nav_msgs::OccupancyGrid Helper::grid;
tf::TransformListener* Helper::listener = NULL;
double Helper::robot_radius;

Helper::Helper()
{
	listener = new(tf::TransformListener);
	posePointCloudClient = nh.serviceClient<ORB_SLAM2::PosePointCloud>("/ORB_SLAM2/posepointcloud");
	pose_sub = nh.subscribe("/vslam/pose_world",100, &Helper::poseCb, this);
	info_sub = nh.subscribe("/vslam/info",100, &Helper::ptamInfoCb, this);
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &Helper::pointCloudCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &Helper::gazeboModelStatesCb, this);
	OccupancyGrid_sub = nh.subscribe("/move_base/global_costmap/costmap", 1, &Helper::OccupancyGridCb, this);
	nh.param("robot_radius", robot_radius, 0.2);
	up = down = left = right = true;
	ros::NodeHandle p_nh("~");
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
sensor_msgs::PointCloud2 Helper::getPointCloud2AtPosition(geometry_msgs::PoseStamped input)
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
pcl::PointCloud<pcl::PointXYZ> Helper::getPCLPointCloudAtPosition(geometry_msgs::PoseStamped input)
{
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	pcl::fromROSMsg(Helper::getPointCloud2AtPosition(input), pointCloud);
	return pointCloud;
}

//Quaternion to RPY
vector<double> Helper::Quat2RPY(geometry_msgs::Quaternion quat)
{
	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(quat, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return {roll, pitch, yaw};
}

//Convert bernstein input in camera frame to pose in world frame
geometry_msgs::PoseStamped Helper::getPoseFromInput(geometry_msgs::PoseStamped input, geometry_msgs::PoseStamped pose)
{
	geometry_msgs::PoseStamped p_out;
	geometry_msgs::Pose currentPose, p, newPose;
	tf::Quaternion currentQuat;
	tf::Pose currentTfPose, newTfPose;

	currentPose = pose.pose;

	p.position.z = input.pose.position.x;
	p.position.x = -input.pose.position.y;
	p.position.y = 0.0;
	p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -Quat2RPY(input.pose.orientation)[2],0.0);
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

//for collision avoidance, can be replaced with any collision avoidance algorithm
bool Helper::inLimits(geometry_msgs::PointStamped point)
{
	float res = grid.info.resolution;
	int cellx = ceil((point.point.x-grid.info.origin.position.x)/res);
	int celly = ceil((point.point.y-grid.info.origin.position.y)/res);
	int cellradius = ceil(robot_radius/res);
	//std::cout<<" Pointx "<<point.point.x<<" Pointy "<<point.point.y<<std::endl;
	//std::cout<<" Cellx "<<cellx<<" celly "<<celly<<std::endl;
	//std::cout<<" current "<<(cellx)+(celly)*grid.info.width<<" dusra "<<(celly)+(cellx)*grid.info.height<<std::endl;
	float data = grid.data[(cellx)+(celly)*grid.info.width];
	float data1 = grid.data[(cellx+cellradius)+(celly)*grid.info.height];
	float data2 = grid.data[(cellx-cellradius)+(celly)*grid.info.height];
	float data3 = grid.data[(cellx)+(celly-cellradius)*grid.info.height];
	float data4 = grid.data[(cellx)+(celly+cellradius)*grid.info.height];
	//std::cout<<" data "<<data<<" data1 "<<data1<<" data2 "<<data2<<" data3 "<<data3<<" data4 "<<data4<<std::endl;
	//if((data < 20)&&(data!=-1))
	//if(data > 20)
	if((data1 > 20) || (data2 > 20) || (data3 > 20) || (data4 > 20))
	{ //std::cout<<"false"<<"\n";
		return false;
	}
	//std::cout<<"true"<<"\n";
	return true;

}

//generate the recovery actions as bernstein inputs
vector<geometry_msgs::PoseStamped > Helper::getPoses()
{
	float angle = PI/90.0, num_angles = 14;
	geometry_msgs::PointStamped point;
	vector<geometry_msgs::PoseStamped > inputs;
	geometry_msgs::PoseStamped pose_odom;
	tf::StampedTransform Tow, Twc;
	tf::Transform Toc;
	try
	{
		//listener->waitForTransform("/odom", "/world", ros::Time::now(), ros::Duration(0.2));
		//listener->transformPose("/odom", pose, pose_odom);
		listener->lookupTransform( "/odom" , "/world" , ros::Time::now() , Tow );
	}
	catch (tf::TransformException &ex)
	{
		try
		{
			listener->lookupTransform( "/odom" , "/world" , ros::Time(0) , Tow );
			ROS_ERROR( " Transform from world to odom frame is unavailable for the time requested. Using latest instead. \n " );
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR_STREAM( " Transform from world to odom frame is unavailable. Error was " << ex.what() << "\n" );
			return inputs;
		}
	}
	tf::poseMsgToTF(pose.pose, Twc);
	Toc = Tow * Twc;
	tf::poseTFToMsg(Toc, pose_odom.pose);
	pose_odom.header.stamp = pose.header.stamp;
	pose_odom.header.frame_id = "/odom";
	double orientation = Quat2RPY(pose_odom.pose.orientation)[2] + 1.57; //converting camera to base_link in orientation
	//std::cout<<" orientation "<< orientation << "position x" << pose_odom.pose.position.x << " y " << pose_odom.pose.position.y <<std::endl;

	for(float i=-num_angles*angle ; i<=num_angles*angle ; i+=angle)
	{
		geometry_msgs::PoseStamped inp;
		inp.header.frame_id = "base_link";
		point.header.stamp = ros::Time::now();
		point.header.frame_id = "odom";

		if(up)
		{
			if(i<0 and !right)
				continue;
			if(i>0 and !left)
				continue;
			point.point.x = pose_odom.pose.position.x + cos(orientation + i);
			point.point.y = pose_odom.pose.position.y + sin(orientation + i);
			if(inLimits(point))
			{	inp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, i);
				inp.pose.position.x = cos(i);
				inp.pose.position.y = sin(i);
				/*inp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0,orientation + i);
				inp.pose.position.x = pose_odom.pose.position.x + cos(orientation + i);
				inp.pose.position.y = pose_odom.pose.position.y + sin(orientation + i);*/
				inp.header.stamp = ros::Time::now();
				inputs.push_back(inp);
			}
		}

		if(down)
		{
			if(i>0 and !left)
				continue;
			if(i<0 and !right)
				continue;
			point.point.x = pose_odom.pose.position.x - cos(orientation - i);
			point.point.y = pose_odom.pose.position.y - sin(orientation - i);
			if(inLimits(point))
			{	inp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, -i);
				inp.pose.position.x = -cos(-i);
				inp.pose.position.y = -sin(-i);
				/*inp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, orientation - i);
				inp.pose.position.x = pose_odom.pose.position.x - cos(orientation - i);
				inp.pose.position.y = pose_odom.pose.position.y - sin(orientation - i);*/
				inp.header.stamp = ros::Time::now();
				inputs.push_back(inp);
			}
		}
	}
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

int Helper::sign(float x)
{
	int val = signbit(x);
	if(val==0)
		return 1;
	else
		return -1;
}

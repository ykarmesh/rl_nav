#include <iostream>
#include <fstream>
#include "Helper.h"
#include <geometry_msgs/PoseArray.h>

using namespace std;

pthread_mutex_t Helper::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::info_mutex = PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t Helper::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Helper::costmap_mutex = PTHREAD_MUTEX_INITIALIZER;

geometry_msgs::PoseStamped Helper::pose;
sensor_msgs::CameraInfo Helper::cam_info;
std_msgs::Bool Helper::ptamInfo;
//geometry_msgs::Pose Helper::robotWorldPose;
sensor_msgs::PointCloud2 Helper::currentPointCloud;
ros::ServiceClient Helper::posePointCloudClient;
bool Helper::up, Helper::down, Helper::left, Helper::right;
nav_msgs::OccupancyGrid Helper::grid;
tf::TransformListener* Helper::listener = NULL;
double Helper::robot_radius;

Helper::Helper()
{
	listener = new(tf::TransformListener);
	posePointCloudClient = nh.serviceClient<ORB_SLAM2::PosePointCloud>("/ORB_SLAM2/posepointcloud");
	pose_sub = nh.subscribe("/vslam/pose_world",500, &Helper::poseCb, this);
	info_sub = nh.subscribe("/vslam/info",100, &Helper::ptamInfoCb, this);
	pointCloud_sub = nh.subscribe("/vslam/frame_points", 100, &Helper::pointCloudCb, this);
	//gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &Helper::gazeboModelStatesCb, this);
	OccupancyGrid_sub = nh.subscribe("/move_base/global_costmap/costmap", 1, &Helper::OccupancyGridCb, this);
	cameraInfo_sub = nh.subscribe("/camera/camera_info", 1, &Helper::cameraInfoCb, this);
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
/*
void Helper::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);
	robotWorldPose = modelStatesPtr->pose.back();
	pthread_mutex_unlock(&gazeboModelState_mutex);
}
*/
void Helper::pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr pointCloudPtr)
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

void Helper::cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr &camerainfo)
{
	cam_info = *camerainfo;
}
//-----------------------------------------------------------------------

//Get ROS pointcloud2 at position relative to camera frame given as bernstein input
sensor_msgs::PointCloud2 Helper::getPointCloud2AtPosition(geometry_msgs::PoseStamped input)
{
  //ORB_SLAM2::PosePointCloud posePointCloud;
  geometry_msgs::PoseStamped pose;
	//PoseStamped from the new point
	sensor_msgs::PointCloud2 pointCloud;
	pthread_mutex_lock(&pose_mutex);
	pose = getPoseFromInput(input, Helper::pose);
	pthread_mutex_unlock(&pose_mutex);
  ros::Time start = ros::Time::now();
	//posePointCloudClient.call(posePointCloud);
	pointCloud = PosePointCloudFunction(pose);
	//return posePointCloud.response.pointCloud;
	return pointCloud;
}

//Get PCL pointcloud at position relative to camera frame given as bernstein input
pcl::PointCloud<pcl::PointXYZ> Helper::getPCLPointCloudAtPosition(geometry_msgs::PoseStamped input)
{
	pcl::PointCloud<pcl::PointXYZ> pointCloud;
	pcl::fromROSMsg(Helper::getPointCloud2AtPosition(input), pointCloud);
	return pointCloud;
}

sensor_msgs::PointCloud2 Helper::PosePointCloudFunction(const geometry_msgs::PoseStamped currentpose)
{
    ros::Time start = ros::Time::now();

    //cv::Mat Ow = -Rpw.t()*tpw;
    tf::Pose Twc, Tcw, Twc0;
		tf::Point Twp, Tcp, Tc0p;
		tf::poseMsgToTF(currentpose.pose,Twc);
		Tcw = Twc.inverse();

		tf::poseMsgToTF(pose.pose, Twc0);

    float fx = cam_info.P[0];
    float fy = cam_info.P[5];
    float cx = cam_info.P[2];
    float cy = cam_info.P[6];

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(currentPointCloud, *cloud);

    sensor_msgs::PointCloud2 pointCloud;

    pointCloud.header.frame_id=MAP_FRAME_ID;
    pointCloud.header.stamp=ros::Time::now();
    pointCloud.header.seq=0;
    pointCloud.fields.resize(3);
    pointCloud.fields[0].name = "x";
    pointCloud.fields[0].offset = 0*sizeof(uint32_t);
    pointCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    pointCloud.fields[0].count = 1;
    pointCloud.fields[1].name = "y";
    pointCloud.fields[1].offset = 1*sizeof(uint32_t);
    pointCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    pointCloud.fields[1].count = 1;
    pointCloud.fields[2].name = "z";
    pointCloud.fields[2].offset = 2*sizeof(uint32_t);
    pointCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    pointCloud.fields[2].count = 1;
    pointCloud.point_step = 3*sizeof(uint32_t);
    pointCloud.is_dense = false;
    pointCloud.data.clear();
    pointCloud.height = 1;
    pointCloud.width = currentPointCloud.width;
    pointCloud.row_step = pointCloud.point_step * pointCloud.width;
    pointCloud.data.resize(pointCloud.row_step * pointCloud.height);
    unsigned char* dat = &(pointCloud.data[0]);
    int num_points = 0;

    for(pcl::PointCloud<pcl::PointXYZ>::iterator vit=cloud->begin(); vit!=cloud->end(); vit++)
    {

	      geometry_msgs::Point P;
				P.x = vit->x;
				P.y = vit->y;
				P.z = vit->z;

        // 3D in camera coordinates
        //const cv::Mat Pc = Rpw*P+tpw;
				tf::pointMsgToTF(P,Twp);
				Tcp = Tcw*Twp;
        const float &PcX = Tcp.getX();
        const float &PcY = Tcp.getY();
        const float &PcZ = Tcp.getZ();
				float dist = sqrt(pow(PcX,2)+pow(PcY,2)+pow(PcZ,2));

				Tc0p = Twc0.inverse()*Twp;
				const float &Pc0X = Tc0p.getX();
				const float &Pc0Y = Tc0p.getY();
  			const float &Pc0Z = Tc0p.getZ();
				float normal = sqrt(pow(Pc0X,2)+pow(Pc0Y,2)+pow(Pc0Z,2));

        // Check positive depth
        if(PcZ<0.01f)
            continue;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<0 || u>cam_info.width)
					continue;
  			if(v<0 || v>cam_info.height)
					continue;

        const float viewCos = (PcX*Pc0X+PcY*Pc0Y+PcZ*Pc0Z)/(normal*dist);

        if(viewCos<0.5)
					continue;

        memcpy(dat, &(vit->x), sizeof(float));
        memcpy(dat+sizeof(float), &(vit->y), sizeof(float));
        memcpy(dat+2*sizeof(float), &(vit->z), sizeof(float));
        num_points++;
        dat+=pointCloud.point_step;
    }
    pointCloud.width=num_points;
    pointCloud.row_step = pointCloud.point_step * pointCloud.width;
    pointCloud.data.resize(pointCloud.row_step * pointCloud.height);
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
		listener->waitForTransform("/odom", "/world", ros::Time::now(), ros::Duration(0.1));
		//listener->transformPose("/odom", pose, pose_odom);
		listener->lookupTransform( "/odom" , "/world" , ros::Time::now() , Tow );
	}
	catch (tf::TransformException &ex)
	{
		try
		{
			listener->lookupTransform( "/odom" , "/world" , ros::Time(0) , Tow );
			ROS_ERROR_STREAM( " Transform from world to odom frame is unavailable for the time requested. "<< ros::Time::now() <<" Using latest instead. " <<ros::Time(0)<< "\n " );
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR_STREAM( " Transform from world to odom frame is unavailable. Error was " << ex.what() << "\n" );
			return inputs;
		}
	}
	pthread_mutex_lock(&pose_mutex);
	tf::poseMsgToTF(pose.pose, Twc);
	pthread_mutex_unlock(&pose_mutex);
	Toc = Tow * Twc;
	tf::poseTFToMsg(Toc, pose_odom.pose);
	pose_odom.header.stamp = pose.header.stamp;
	pose_odom.header.frame_id = "/odom";
	double orientation = Quat2RPY(pose_odom.pose.orientation)[2] + 1.57; //converting camera to base_link in orientation


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

#include <algorithm>
#include <limits>
#include <numeric>
#include <functional>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <utility>
#include <tuple>

#include <cmath>
#include <ctime>
#include <cstdlib>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/CameraInfo.h>
//#include <ptam_com/ptam_info.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <ORB_SLAM2/PosePointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <gazebo_msgs/ModelStates.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

using namespace std;

const long double PI = 3.141592653589793238L;

#define min(a,b) (a<b)?a:b
#define max(a,b) (a>b)?a:b

class Helper
{
	struct pointEqComparer
	{
		bool operator()(const pcl::PointXYZ &p_left, const pcl::PointXYZ &p_right)
		{
			return p_left.x == p_right.x and p_left.y == p_right.y and p_left.z == p_right.z;
		}
	};

	struct pointLtComparer
	{
		bool operator()(const pcl::PointXYZ &p_left, const pcl::PointXYZ &p_right)
		{
			return p_left.x < p_right.x and p_left.y < p_right.y and p_left.z < p_right.z;
		}
	};

	static pthread_mutex_t pose_mutex, info_mutex, pointCloud_mutex, costmap_mutex; //gazeboModelState_mutex

	static geometry_msgs::PoseStamped pose;
	static sensor_msgs::CameraInfo cam_info;
	static std_msgs::Bool ptamInfo;
	static geometry_msgs::Pose robotWorldPose;
	static sensor_msgs::PointCloud2 currentPointCloud;

	ros::NodeHandle nh;
	ros::Subscriber pose_sub, info_sub, pointCloud_sub, OccupancyGrid_sub, cameraInfo_sub; //gazeboModelStates_sub
	static ros::ServiceClient posePointCloudClient;

	void poseCb(const geometry_msgs::PoseStampedPtr posePtr);
	//void ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr);
	void ptamInfoCb(const std_msgs::BoolPtr ptamInfoPtr);
	//void gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr);
	void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr pointCloudPtr);
	void OccupancyGridCb(const nav_msgs::OccupancyGrid::ConstPtr &OGPtr);
        void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr &Camerainfo);

public:
	Helper();
	static sensor_msgs::PointCloud2 getPointCloud2AtPosition(geometry_msgs::PoseStamped input);
	static pcl::PointCloud<pcl::PointXYZ> getPCLPointCloudAtPosition(geometry_msgs::PoseStamped input);
        static sensor_msgs::PointCloud2 PosePointCloudFunction(const geometry_msgs::PoseStamped currentpose);
	static vector<double> Quat2RPY(geometry_msgs::Quaternion quat);
	static geometry_msgs::PoseStamped getPoseFromInput(geometry_msgs::PoseStamped input, geometry_msgs::PoseStamped pose);
	static vector<pcl::PointXYZ> pointCloudIntersection(pcl::PointCloud<pcl::PointXYZ> pointCloudA, pcl::PointCloud<pcl::PointXYZ> pointCloudB);
	static bool inLimits(geometry_msgs::PointStamped point);
	static vector<geometry_msgs::PoseStamped > getPoses();
	static void saveFeatureExpectation(vector<vector<vector<int> > > episodeList, string fileName);
	static vector<vector<vector<int> > > readFeatureExpectation(string fileName);
	static int sign(float x);
	static nav_msgs::OccupancyGrid grid;
	static tf::TransformListener* listener;
	static double robot_radius;

	static constexpr char* MAP_FRAME_ID = "world";
	static constexpr char* NAV_FRAME_ID = "world2D";
	static constexpr char* GAZEBO_FRAME_ID = "base_link";
    static constexpr char* CAMERA_FRAME_ID = "camera_rgb_optical_frame";

	static bool up, down, left, right;
};

#include <algorithm>
#include <limits>
#include <numeric>
#include <functional>
#include <thread>

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>

#include "Helper.h"
#include "JoystickNode.h"

#include <tf/transform_datatypes.h>

#include <rl_nav/ExpectedPath.h>

using namespace std;

pthread_mutex_t JoystickNode::pose_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::pointCloud_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::ptamInfo_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::plannerStatus_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::gazeboModelState_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t JoystickNode::globalPlanner_mutex = PTHREAD_MUTEX_INITIALIZER;
//tf::TransformBroadcaster JoystickNode::tfBroadcaster;

JoystickNode::JoystickNode()
{
	srand (time(NULL));
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);
	goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
	planner_reset_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);
	gazebo_state_reset_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);
	ptam_com_pub = nh.advertise<std_msgs::String>("/vslam/key_pressed",1);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/my_pose",1);
	next_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/my_next_pose",1);
	next_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/my_next_pc",1);
	expected_pub = nh.advertise<geometry_msgs::PoseStamped>("/expected_pose",1);
	ptam_path_pub = nh.advertise<visualization_msgs::Marker>("/vslam_path",1);
	gazebo_path_pub = nh.advertise<visualization_msgs::Marker>("/gazebo_path",1);
	gazebo_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gazebo_pose",1);
	init_pub = nh.advertise<std_msgs::Empty>("/rl/init",1);
	sendCommand_pub = nh.advertise<std_msgs::Empty>("/rl/sendCommand",1);
	safe_poses_pub = nh.advertise<geometry_msgs::PoseArray>("/rl/safe_poses",1);
	unsafe_poses_pub = nh.advertise<geometry_msgs::PoseArray>("/rl/unsafe_poses",1);
	odom_reset_pub = nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",1);

	expectedPathClient = nh.serviceClient<rl_nav::ExpectedPath>("/planner/global/expected_path");
	num=0;
	ifstream ratioFile("ratioFile.txt");
	ratioFile >> rlRatio >> num_episodes;
	if(!rlRatio)
		rlRatio = 10;
	ratioFile.close();
	episodeList = Helper::readFeatureExpectation("tempfeFile.txt");
	state = 0;
	breakCount = 0;
	joy.buttons = vector<int>(11,0);
	joy.axes = vector<float>(8,0);
	prevQ = 1;
	initState.model_name = "";
	just_init = false;
	initialized = false;
	initY = 0;
	num_broken = 0;
	Q_THRESH = 0;
	up = down = left = right = true;
	vel_scale = 1.0;
	listener = new(tf::TransformListener);

	actionClient = new MoveBaseClient(string("move_base"), true);
	//wait for the action server to come up
	while(!actionClient->waitForServer(ros::Duration(5.0)))
		ROS_INFO("Waiting for the move_base action server to come up");



	qFile.open(string("qData.txt"),ios::app);

	joy_sub = nh.subscribe("/joy", 100, &JoystickNode::joyCb, this);
	init_sub = nh.subscribe("/rl/init", 100, &JoystickNode::initCb, this);
	sendCommand_sub = nh.subscribe("/rl/sendCommand", 100, &JoystickNode::sendCommandCb, this);
	pose_sub = nh.subscribe("/vslam/pose_world",100, &JoystickNode::poseCb, this);
	//pointCloud_sub = nh.subscribe("/vslam/pc2", 100, &JoystickNode::pointCloudCb, this);
	ptamInfo_sub = nh.subscribe("/vslam/info", 100, &JoystickNode::ptamInfoCb, this);
	ptamStart_sub = nh.subscribe("/vslam/started", 100, &JoystickNode::ptamStartedCb, this);
	plannerStatus_sub = nh.subscribe("/planner/status", 100, &JoystickNode::plannerStatusCb, this);
	local_plan_sub = nh.subscribe("/move_base/TebLocalPlannerROS/local_plan", 100, &JoystickNode::globalNextPoseCb, this);
	//globalPoints_sub = nh.subscribe("/planner/global/path", 100, &JoystickNode::globalNextPoseCb, this);
	gazeboModelStates_sub = nh.subscribe("/gazebo/model_states", 100, &JoystickNode::gazeboModelStatesCb, this);
	waypoint_sub = nh.subscribe("/move_base_simple/waypoint",100,&JoystickNode::waypointCb,this);
	goal_sub = nh.subscribe("/move_base_simple/goal",100,&JoystickNode::goalCb,this);

	vslam_path.id=0;
	vslam_path.lifetime=ros::Duration(1);
	vslam_path.header.frame_id = "/world";
	vslam_path.header.stamp = ros::Time::now();
	vslam_path.ns = "pointcloud_publisher";
	vslam_path.action = visualization_msgs::Marker::ADD;
	vslam_path.type = visualization_msgs::Marker::LINE_STRIP;
	vslam_path.color.r=0.0;
	vslam_path.color.b=1.0;
	vslam_path.color.a=1.0;
	vslam_path.scale.x=0.1;
	vslam_path.pose.orientation.w=1.0;

	gazebo_path.id=0;
	gazebo_path.lifetime=ros::Duration(1);
	gazebo_path.header.frame_id = "world2D";
	gazebo_path.header.stamp = ros::Time::now();
	gazebo_path.ns = "pointcloud_publisher";
	gazebo_path.action = visualization_msgs::Marker::ADD;
	gazebo_path.type = visualization_msgs::Marker::LINE_STRIP;
	gazebo_path.color.r=1.0;
	gazebo_path.color.g=0.0;
	gazebo_path.color.a=1.0;
	gazebo_path.scale.x=0.1;
	gazebo_path.pose.orientation.w=1.0;

	initState.reference_frame = "world";
	initState.pose.position.z = 0;

	MAP=-1;
	ros::NodeHandle p_nh("~");
	p_nh.getParam("qThresh", Q_THRESH);
	p_nh.getParam("mode", MODE);
	p_nh.getParam("num_episodes", MAX_EPISODES);
	p_nh.getParam("max_steps", MAX_STEPS);
	p_nh.getParam("init_x", initState.pose.position.x);
	p_nh.getParam("init_y", initState.pose.position.y);
	p_nh.getParam("init_Y", initYaw);
	p_nh.getParam("map", MAP);
	p_nh.getParam("vel_scale", vel_scale);
	p_nh.getParam("init_angle", INIT_ANGLE);
	p_nh.getParam("local_planner", LOCAL_PLANNER);


	initState.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, initYaw);
	learner.clear();
	//TEST mode means testing steps to failure
	//In steps to failure mode, the agent directly starts off
	if(!MODE.compare("TRAIN") or !MODE.compare("TEST"))
	{
		state = 1;
		initCb(std_msgs::Empty());
	}
	else if(!MODE.compare("MAP"))
		state = 2;
}

JoystickNode::~JoystickNode()
{
	qFile.close();
	//cout<<"Quitting JoystickNode"<<endl;
	ros::NodeHandle p_nh("~");
	p_nh.deleteParam("qThresh");
	p_nh.deleteParam("mode");
	p_nh.deleteParam("num_episodes");
	p_nh.deleteParam("max_steps");
	p_nh.deleteParam("init_x");
	p_nh.deleteParam("init_y");
	p_nh.deleteParam("init_Y");
	p_nh.deleteParam("map");
	p_nh.deleteParam("vel_scale");
	p_nh.deleteParam("init_angle");

	ofstream ratioFile("ratioFile.txt");
	ratioFile << ((rlRatio==90)?10:rlRatio) << " " << num_episodes << endl;
	ratioFile.close();
}

//In case SLAM crashes and restarts, restart the RL agent as well
void JoystickNode::ptamStartedCb(const std_msgs::EmptyPtr emptyPtr)
{
	episode.clear();
	initCb(std_msgs::Empty());
}

/**
 *	PTAM Initializer callback
 */
void JoystickNode::initCb(const std_msgs::Empty empty)
{
	if(!MODE.compare("TRAIN"))
	{
		initState.pose.position.x = -rand()%5 - 1;
		initState.pose.position.y = rand()%3 + 0.5;
		initState.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, (rand()%360)*PI/180);
	}
	std_msgs::String resetString, spaceString;
	geometry_msgs::Twist twist;
	initY = 0;
	planner_reset_pub.publish(actionlib_msgs::GoalID());//stop planner
	gazebo_path.points.clear();
	vslam_path.points.clear();
	resetString.data = "r";
	spaceString.data = "Space";
	//reset SLAM before initializing
	//done 4 times just in case SLAM doesn't get resetin one time
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);
	ptam_com_pub.publish(resetString);

	//reseting robot position in gazebo and robot odometry
	gazebo_state_reset_pub.publish(initState);
	odom_reset_pub.publish(std_msgs::Empty());
	//ptam_com_pub.publish(spaceString);

	//moving the robot back for a small distance to get the baseline for initialization
	twist.linear.x=-0.4;
	clock_t t = clock();
	ros::Rate r(10);
	while(((float) (clock() - t))/CLOCKS_PER_SEC < 0.15)
	{
		vel_pub.publish(twist);
		r.sleep();
	}

	num_broken = 0;
	just_init=true;
	initialized = false;
	ros::Rate(1).sleep();
	//ptam_com_pub.publish(spaceString);
}

/**
 *	Receive PTAM pose of camera in world
 */
void JoystickNode::poseCb(const geometry_msgs::PoseStampedPtr posePtr)
{
	pthread_mutex_lock(&pose_mutex);
	pose = *posePtr;
	pose.header.frame_id = "world";

	vector<double> orientation;
	double angle;
	//if SLAM is just initialized
	if(just_init)
	{
		just_init=false;
		orientation = Helper::Quat2RPY(pose.pose.orientation);
		angle = abs(orientation[1]); //empirically observed
		//cout<<orientation[0]<<" "<<orientation[1]<<" "<<orientation[2]<<" "<<(orientation[0]-3.14)*(orientation[0]-3.14)<<endl;

		if((angle-INIT_ANGLE)*(angle-INIT_ANGLE) > 0.003) //INIT_ANGLE is also empirically estimated
			initCb(std_msgs::Empty());	//if SLAM isn't initialized properly, reinitialize it
		else
		{
			initialized = true;
			gazebo_path.points.clear();
			vslam_path.points.clear();
			if(state==1)
				sendCommandCb(std_msgs::Empty());
			/*else if(state==2)
				goal_pub.publish(goalPose);*/
		}
	}
	else
		initialized = true;

	if(!initY)
		initY = pose.pose.position.y;

	if((initY - pose.pose.position.y)*(initY - pose.pose.position.y) >=0.15) //SLAM gets initialized in the XZ plane
		num_broken++;														 //if Y coordinate is fluctuating then estimation is bad
	else if(num_broken>0)
		num_broken--;

	geometry_msgs::PoseStamped ps = pose;

	//The default pose points 90 deg away from the actual pose to the right
	//This corrects that to point in the camera direction
	vector<double> curr_angles = Helper::Quat2RPY(pose.pose.orientation);
	if(curr_angles[2]*curr_angles[2] > (PI - fabs(curr_angles[2]))*(PI - fabs(curr_angles[2])))
		ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(curr_angles[0], PI/2.0 + curr_angles[1], curr_angles[2]);
	else
		ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(curr_angles[0], -PI/2.0 + curr_angles[1], curr_angles[2]);
	pose_pub.publish(ps);


	//vslam_path.points.push_back(pose.pose.position);
	vslam_path.points.push_back(ps.pose.position);
	ptam_path_pub.publish(vslam_path);

	gazebo_path.points.push_back(robotWorldPose.position);
	gazebo_path_pub.publish(gazebo_path);

	pthread_mutex_unlock(&pose_mutex);
}

/**
 *	Receive next expected robot pose w.r.t. current pose along global path
 */
void JoystickNode::globalNextPoseCb(const nav_msgs::PathPtr pathPtr)
{
	pthread_mutex_lock(&globalPlanner_mutex);
	geometry_msgs::Pose pose, prev_pose;
	tf::Pose To1, To2, T12;

	float distance = 0;
	int iter = 0;
	prev_pose.position.x = pathPtr->poses[0].pose.position.x;
	prev_pose.position.y = pathPtr->poses[0].pose.position.y;
	tf::poseMsgToTF(pathPtr->poses[0].pose, To1);
	for(int i=0; i < pathPtr->poses.size(); i++)
	{
		pose = pathPtr->poses[i].pose;
		distance+= sqrt(pow((pose.position.x-prev_pose.position.x),2)+pow((pose.position.y-prev_pose.position.y),2));
		iter = i;
		if(distance>1)
		{
			break;
		}
		prev_pose = pose;
	}
	expected_pose.header.stamp = ros::Time::now();
	expected_pose.header.frame_id = "/base_link";
	tf::poseMsgToTF(pose, To2);
	T12 = To1.inverse()*To2;
	tf::poseTFToMsg(T12, expected_pose.pose);

	//std::cout<<" temp_pose.x "<<temp_pose.position.x<<" temp_pose.y "<<temp_pose.position.y<<" temp_pose.yaw "<<Helper::Quat2RPY(temp_pose.orientation)[2]<<std::endl;
	/*try
	{
		listener->waitForTransform("/base_link", "/odom", ros::Time::now(), ros::Duration(0.1));
		listener->transformPose("/base_link", temp, expected_pose);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s",ex.what());
		pthread_mutex_unlock(&globalPlanner_mutex);
		return;
	}*/
	//std::cout<<" expected_pose.x "<<expected_pose.pose.position.x<<" expected_pose.y "<<expected_pose.pose.position.y<<" expected_pose.yaw "<<Helper::Quat2RPY(expected_pose.pose.orientation)[2]<<std::endl;
	float Q; //Q value of the last state-action pair
	vector<int> stateAction;
	tie(ignore, stateAction, Q) = learner.getAction(expected_pose); //convert the subsequent part of the trajectory into a RL state-action pair

	//ptam_com::ptam_info info;
	std_msgs::Bool info;

	pthread_mutex_lock(&ptamInfo_mutex);
	info = ptamInfo;
	pthread_mutex_unlock(&ptamInfo_mutex);

	//publish the next expected pose and pointcloud
	next_pose_pub.publish(expected_pose);
	next_pc_pub.publish(Helper::getPointCloud2AtPosition(expected_pose));

	//if SLAM broke
	//if(!info.trackingQuality)
	if(!info.data)
	{
		planner_reset_pub.publish(actionlib_msgs::GoalID());//stop planner
		cout<<"UH OH!!!"<<endl<<"Breaking after:\t";
		for(auto i: stateAction)
			cout<<i<<'\t';
		cout<<Q<<endl;
	}
	//else if(!MODE.compare("MAP") and learner.predict(stateAction))
	else if(!MODE.compare("MAP") and Q < Q_THRESH) //if a failure is predicted
	{
		state = 1;
		cout<<"predicted break "<< prevQ<<endl;
		for(auto i: stateAction)
			cout<<i<<'\t';
		cout<<endl;
		planner_reset_pub.publish(actionlib_msgs::GoalID());//stop planner
		planner_reset_pub.publish(actionlib_msgs::GoalID());//stop planner
		learner.clear();
		sendCommandCb(std_msgs::Empty());
	}
	/*else cout<<"Q VALUE: "<<Q<<endl;*/

	pthread_mutex_unlock(&globalPlanner_mutex);
}

/**
 *	Receive joystick input
 */
void JoystickNode::joyCb(const sensor_msgs::JoyPtr joyPtr)
{
	geometry_msgs::Twist command;
	if(joyPtr->buttons[POWER] and !joy.buttons[POWER])
		cout << breakCount << " " << rlRatio << " " << num_steps << " " << num_episodes <<endl;
	//arrows on the xbox controller
	//pressing an arrow toggles the generation of recovery actions in that direction
	if((joyPtr->axes[DH] and !joy.axes[DH]) and joyPtr->axes[DH] != joy.axes[DH])
	{
		if(joyPtr->axes[DH]==-1)
		{
			Helper::right = not Helper::right;
			if(Helper::left == false and Helper::right == false)
				Helper::left = true;
		}
		else if(joyPtr->axes[DH]==1)
		{
			Helper::left = not Helper::left;
			if(Helper::left == false and Helper::right == false)
				Helper::right = true;
		}
	}
	else if((joyPtr->axes[DV] and !joy.axes[DV]) and joyPtr->axes[DV] != joy.axes[DV])
	{
		if(joyPtr->axes[DV]==1)
		{
			Helper::up = not Helper::up;
			if(Helper::up == false and Helper::down == false)
				Helper::down = true;
		}
		else if(joyPtr->axes[DV]==-1)
		{
			Helper::down = not Helper::down;
			if(Helper::up == false and Helper::down == false)
				Helper::up = true;
		}
	}
	//pressing RB resets SLAM
	else if(joyPtr->buttons[RB] and !joy.buttons[RB])
	{
		std_msgs::String resetString;
		resetString.data = "r";
		ptam_com_pub.publish(resetString);
	}
	//pressing LB is the PTAM equivalent of pressing space
	else if(joyPtr->buttons[LB] and !joy.buttons[LB])
	{
		std_msgs::String spaceString;
		spaceString.data = "Space";
		ptam_com_pub.publish(spaceString);
	}
	//Start button initializes SLAM
	else if(joyPtr->buttons[START] and !joy.buttons[START])
		initCb(std_msgs::Empty());
	//Back/Select button shutsdown the node
	else if(joyPtr->buttons[BACK] and !joy.buttons[BACK])
		ros::shutdown();
	//pressing A executes a recovery action
	else if(joyPtr->buttons[A] and !state)
	{
		//send input to planner
		state = 1;
		sendCommandCb(std_msgs::Empty());
	}
	//B stops the planner
	else if(joyPtr->buttons[B]  and !joy.buttons[B])
	{
		if(state)
			planner_reset_pub.publish(actionlib_msgs::GoalID());//stop planner
		state = 0;
		num_broken = 0;
		breakCount = 0;
	}
	//Y resets the below values
	else if(joyPtr->buttons[Y])
	{
		num_broken = 0;
		breakCount = 0;
	}
	//X starts the global path (ONLY TO BE USED IN MAP MODE)
	else if(joyPtr->buttons[X] and !joy.buttons[X])
	{
		state = 2;
		goal_pub.publish(goalPose);
	}
	//Left analog for fwd/bkwd, Right analog for yaw
	else if((fabs(joyPtr->axes[LV])>0.009 or fabs(joyPtr->axes[RH])>0.009) and !state)
	{
		command.linear.x = joyPtr->axes[LV]*vel_scale;
		command.angular.z = joyPtr->axes[RH]*vel_scale;
		vel_pub.publish(command);
	}
	joy = *joyPtr;
}

/**
 *	Receive PTAM point cloud
 */
void JoystickNode::pointCloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr)
{
	pthread_mutex_lock(&pointCloud_mutex);
	pointCloud = *pointCloudPtr;
	pthread_mutex_unlock(&pointCloud_mutex);
}

/**
 *	Receive PTAM Info
 */
//void JoystickNode::ptamInfoCb(const ptam_com::ptam_infoPtr ptamInfoPtr)
 void JoystickNode::ptamInfoCb(const std_msgs::BoolPtr ptamInfoPtr)
{
	pthread_mutex_lock(&ptamInfo_mutex);
	ptamInfo = *ptamInfoPtr;
	pthread_mutex_unlock(&ptamInfo_mutex);
}

/**
 *	Receive Gazebo Models ground truth data
 */
void JoystickNode::gazeboModelStatesCb(const gazebo_msgs::ModelStatesPtr modelStatesPtr)
{
	pthread_mutex_lock(&gazeboModelState_mutex);
	robotWorldPose = modelStatesPtr->pose.back();
	if(just_init and not initState.model_name.length() and modelStatesPtr->name.back().length())
		initCb(std_msgs::Empty());
	initState.model_name = modelStatesPtr->name.back();

//ERROR, this is not the world2D frame
	//publish the ground truth pose
	geometry_msgs::PoseStamped ps;
	ps.header.stamp = ros::Time::now();
	ps.header.frame_id = "world2D";
	ps.pose = robotWorldPose;
	gazebo_pose_pub.publish(ps);


	pthread_mutex_unlock(&gazeboModelState_mutex);
}

/**
 *	Receive Planner status after executing local action (recovery action)
 */
void JoystickNode::plannerStatusCb(const std_msgs::StringPtr plannerStatusPtr)
{
	pthread_mutex_lock(&plannerStatus_mutex);

	if(!(plannerStatusPtr->data.compare("DONE")))
	{

	}

	pthread_mutex_unlock(&plannerStatus_mutex);

}
/**
 * Receiving the Goal Pose from rviz
 */
void JoystickNode::goalCb(const geometry_msgs::PoseStampedPtr goalPosePtr)
{
	goalPose = *goalPosePtr;
}

/**
 *	Receiving the waypoint
 */
void JoystickNode::waypointCb(const geometry_msgs::PoseStampedPtr waypointPosePtr)
{
	waypointPose = waypointPosePtr->pose;
}

/**
 *	Sending commands to the robot
 */
void JoystickNode::sendCommandCb(std_msgs::Empty empty)
{
	if(initialized)//will work only if SLAM is initialized
	{
		geometry_msgs::PoseArray safe_poses, unsafe_poses;
		float Q;
		vector<int> RLInput; //last RL Input

		//get the safe and unsafe trajectories and visualize them
		for(auto pose: Helper::getPoses())
		{
			tie(ignore, ignore, Q) = learner.getAction(pose);
			if(Q>Q_THRESH)
				safe_poses.poses.push_back(pose.pose);
			else
				unsafe_poses.poses.push_back(pose.pose);
		}

		safe_poses.header.frame_id ="base_link";
		unsafe_poses.header.frame_id ="base_link";
		safe_poses.header.stamp = ros::Time::now();
		unsafe_poses.header.stamp = ros::Time::now();
		safe_poses_pub.publish(safe_poses);
		unsafe_poses_pub.publish(unsafe_poses);

		//incremental training epsilon greedy
		if(!MODE.compare("TRAIN"))
			tie(lastPose, RLInput, prevQ) = learner.getEpsilonGreedyStateAction(rlRatio,lastPose);
		else if(!MODE.compare("TEST"))
			tie(lastPose, RLInput, prevQ) = learner.getEpsilonGreedyStateAction(95,lastPose);
		else if(!MODE.compare("MAP"))
		{
/*			rl_nav::ExpectedPath expectedPath;
			expectedPathClient.call(expectedPath);
			vector<float> poses = expectedPath.response.expectedPath.data;
			float nextAngle = atan(poses[5]);
*/			//float nextAngle = atan2 (waypointPose.position.y + pose.pose.position.x, waypointPose.position.x + pose.pose.position.z);
			//float nextAngle = atan2 (waypointPose.position.y + robotWorldPose.position.x, waypointPose.position.x + robotWorldPose.position.z);
			/*geometry_msgs::Pose prev_pose;
			tf::Pose To1, To2, T12;
			tf::poseMsgToTF(expected_pose.pose, Tbe);
			tf::poseMsgToTF(pose.pose, Twc);
			T12 = To1.inverse()*To2;
			tf::poseTFToMsg(T12, expected_pose.pose);*/
			geometry_msgs::PoseStamped expected_pose_w2D;
			try
			{
				listener->waitForTransform("/world2D", "/base_link", ros::Time::now(), ros::Duration(0.1));
				listener->transformPose("/world2D", expected_pose, expected_pose_w2D);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
				return;
			}
			float nextAngle = Helper::Quat2RPY(expected_pose_w2D.pose.orientation)[2];

			tie(lastPose, RLInput, prevQ) = learner.getThresholdedClosestAngleStateAction(Q_THRESH, nextAngle, lastPose);
			//tie(lastPose, RLInput, prevQ) = learner.getSLClosestAngleStateAction(nextAngle);
		}
		num_steps++;

		//publish the next pose and pointcloud
		next_pose_pub.publish(lastPose);
		next_pc_pub.publish(Helper::getPointCloud2AtPosition(lastPose));

		//send the next trajectory to the planner
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose = lastPose;
		learner.clear();
		actionClient->sendGoal(goal);

		actionClient->waitForResult();

		if(actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) //if the action succeeded
		{
			breakCount++;
			//ptam_com::ptam_info info;
			std_msgs::Bool info;

			pthread_mutex_lock(&ptamInfo_mutex);
			info = ptamInfo;
			pthread_mutex_unlock(&ptamInfo_mutex);
			//if(!episode.size() or (episode.size() and RLInput!=episode.back()))
			//{
			if(RLInput.size()==3)
				//RLInput.push_back((num_broken <= 3 and info.trackingQuality)?0:1);
				RLInput.push_back((num_broken <= 3 and info.data)?0:1);
				for(auto i: RLInput)
					qFile<<i<<'\t';
				//write all the possible recovery actions from the current state ot file as well, separated by ';'
				/*qFile<<';';
				for(auto input : Helper::getPoses())
				{
					vector<int> rlInput;
					float Q;
					tie(ignore, rlInput, Q) = learner.getAction(input);
					for(auto i : rlInput)
						qFile<<i<<'\t';
					qFile<<';';
				}*/
				qFile << '\n';
				//episode.push_back(RLInput);
			//}
			//learner.updateQ(RLInput,get<1>(learner.getBestQStateAction(lastPose))); //update the Q values after performing a recovery action
			//if(num_broken>3 or !info.trackingQuality)
			if(num_broken>3 or !info.data) //if SLAM is broken
			{
				cout<<"Breaking after "<<breakCount<< " steps due to action with Q value "<< prevQ<<'\t';
				for(auto i: RLInput)
					cout<<i<<'\t';
				cout<< endl;
				breakCount = 0;
				initialized = false;

				//episodeList.push_back(episode);
				//episode.clear();
				num_episodes++;

				if(MODE.compare("MAP") and num_episodes == MAX_EPISODES)//if the maximum number of episodes have been reached
				{
					if(!MODE.compare("TRAIN"))//in training phase, incrememnt the exploitation ratio
					{
						learner.episodeUpdate(episodeList);
						episodeList.clear();
						rlRatio+=10;
						num_steps = 0;
						num_episodes = 0;
						if(rlRatio==90)
						{
							//cout<<"SWITCHING TO TESTING PHASE"<<endl;
							//MODE = "TEST";/
							ros::shutdown();
						}
					}
					else if(!MODE.compare("TEST"))//in testing phase, save the trajectories and shutdown
					{
						Helper::saveFeatureExpectation(episodeList, "feFile.txt");
						remove("tempfeFile.txt");
						ros::shutdown();
					}
				}
				if(!MODE.compare("MAP"))//if SLAM breaks in MAP mode, stop the planner
					planner_reset_pub.publish(actionlib_msgs::GoalID());//stop planner
				else //else reinitialize and start again
					initCb(std_msgs::Empty());

			}
			else if(state==1) //if SLAM is not broken and it was in recovery action state
			{
				if(!MODE.compare("MAP"))//if it's in MAP mode, continue along the global trajectory
				{
					state = 2;
					breakCount = 0;
					goal_pub.publish(goalPose);
				}
				else //if it's in TRAIN or TEST modes, execute the next step (recovery atcion)
					sendCommandCb(std_msgs::Empty());

			}
		}
		else //if the action was unsuccessful, try again
			sendCommandCb(std_msgs::Empty());
	}
}

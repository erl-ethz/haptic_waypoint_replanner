#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <string.h>
#include <math.h>
#include <algorithm> 
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <tf/transform_datatypes.h>
#include <vector>
#include <numeric>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <ros/duration.h>
#include <Eigen/Dense>
#include <quadrotor_common/geometry_eigen_conversions.h>

using namespace std;

//////////////////// Global variables ////////////////////

// Publishers and subscribers
ros::Subscriber odom_sub;				// Subscriber to odometry
ros::Subscriber ft_sensor_sub;				// Subscriber to F/T sensor
ros::Subscriber start_cmd_sub;				// Start command sub
ros::Subscriber ref_pose_sub;				// Subscriber to reference pose command
ros::Publisher  ref_pose_pub;				// Publish reference position
ros::Publisher  log_ft_pub;				// Publish compensated F/T data for logging
ros::Publisher  ft_filtered_pub;			// Publisher for filtered F/T data
ros::Publisher  state_pub;				// Publisher for current state of the state machine
ros::Publisher  ref_pose_log_pub;			// Publisher for reference position log

// UAV variables
Eigen::Vector3f position;
Eigen::Vector3f ref_position(0.0, 0.0, 0.0);
Eigen::Vector4f ref_orientation(0.0, 0.0, 0.0, 1.0);
tf::Quaternion orientation;
double roll, pitch, yaw;
geometry_msgs::PoseStamped ref_pose_msg;
geometry_msgs::Pose ref_pose_log_msg;			// Reference pose log

// Force/torque variables
geometry_msgs::Wrench log_ft_msg;
Eigen::Vector3f raw_forces, raw_torques;
Eigen::Vector3f bias_forces(0.0, 0.0, 0.0);
Eigen::Vector3f bias_torques(0.0, 0.0, 0.0);
int j = 0;
double force_magnitude;
double polar_angle, azimuth_angle;
double MIN_VERTICAL_FORCE_THRESHOLD;			// [N]
double MAX_VERTICAL_FORCE_THRESHOLD;	 		// [N]

// Moving Average Filter variables
geometry_msgs::Wrench ft_filtered_msg;
Eigen::Vector3f filtered_forces, filtered_torques;
const int N = 75;				  	// Window's size
double forces_cum[3][N];
double torques_cum[3][N];
Eigen::Vector3f sum_forces, sum_torques;
bool filter_flag = true;

// Initialization and execution variables
bool start_flag = false;
bool init_flag = false;
double publishing_time;
const int PUB_TIME = 1;				  	// New waypoint (while pushing down) is published every 'PUB_TIME' seconds
double init_time;
const int INIT_SAMPLES = 250;				// After 'INIT_SAMPLES' iterations, the biases are computed

// Waypoint Replanner variables
std_msgs::Int32 state_msg;
int state = 0;
Eigen::Vector3f landing_position, interaction_position, control_position;
double initial_yaw, interaction_yaw, control_yaw;
double commanded_polar, commanded_azimuth;
double resting_time;
int RESTING_TIME;					// The drone will rest for 'RESTING_TIME' seconds	
int steps = 1;
double homing_time;
const int HOME_TIME = 3;
double first_interaction_x, first_interaction_y;
double lateral_slide_x      = 0.0;
double lateral_slide_y      = 0.0;
double MAX_LATERAL_SLIDING  = 0.1;	  		// [m]
double pushing_effort;
double last_effort;
double landing_gain;
double DELTA_PUSH;
const double MAX_EFFORT     = 30;
double control_gain_x; 	 			 	// Controller Gain for X component
double control_gain_y; 	 			 	// Controller Gain for Y component
double control_gain_z; 	 			 	// Controller Gain for Z component
bool landing_flag	    = false;
bool contact_flag	    = false;
bool sliding_flag           = false;
int outdoor;
bool outdoor_flag = false;

//////////////////////////////////////// Callbacks ////////////////////////////////////////

// Start button callback
void startCallback(const std_msgs::Bool::ConstPtr& start_msg)
{
	if(start_msg->data)
  	{
		start_flag = true;
		if(j == 0)
		{
			cout << "\033[32m\033[1mHWR: start command received! \033[0m" << endl;
			init_time = ros::Time::now().toSec();
			j++;
		}
	}
	else
	{
 		start_flag = false;
		state = 0;
		landing_flag   = false;
		sliding_flag   = false;
		contact_flag   = false;
		cout << "\033[32m\033[1mMISSION ABORTED! \033[0m" << endl;
	}
}

// State estimator odometry callback
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	// Position
	position[0] = odom_msg->pose.pose.position.x;
	position[1] = odom_msg->pose.pose.position.y;
	position[2] = odom_msg->pose.pose.position.z;
	
	// Orientation
	tf::Quaternion q_body(odom_msg->pose.pose.orientation.x,
			      odom_msg->pose.pose.orientation.y,
			      odom_msg->pose.pose.orientation.z,
			      odom_msg->pose.pose.orientation.w);
	tf::Matrix3x3 m_b(q_body);
	m_b.getRPY(roll, pitch, yaw);	

	orientation = q_body;

	// Compute displacement if the current position change
	if(!start_flag)
		initial_yaw = yaw;
}

// Reference pose commands callback
void refPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& ref_pose_cmd_msg)
{
	ref_position[0] = ref_pose_cmd_msg->pose.position.x;
	ref_position[1] = ref_pose_cmd_msg->pose.position.y;
	ref_position[2] = ref_pose_cmd_msg->pose.position.z;

	ref_orientation[0] = ref_pose_cmd_msg->pose.orientation.x;
	ref_orientation[1] = ref_pose_cmd_msg->pose.orientation.y;
	ref_orientation[2] = ref_pose_cmd_msg->pose.orientation.z;
	ref_orientation[3] = ref_pose_cmd_msg->pose.orientation.w;
}

// Callback for force/torque measurement acquisition from FT sensor
void forceTorqueSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& ft_msg)
{
	// Read raw forces/torques data and convert in the correct reference frame
	raw_forces[0]  =  ft_msg->wrench.force.x;
	raw_forces[1]  = -ft_msg->wrench.force.y;
	raw_forces[2]  = -ft_msg->wrench.force.z;
	raw_torques[0] =  ft_msg->wrench.torque.x;
	raw_torques[1] = -ft_msg->wrench.torque.y;
	raw_torques[2] = -ft_msg->wrench.torque.z;
}

//////////////////////////////////////// Functions ////////////////////////////////////////

// Function to inizialize the biases of the load cell when the drone is in hovering, ready to start landing
void InitializeBiases()
{
	// Acquire 'INIT_SAMPLES' (number) samples
	int i = 0;
	vector<float> sum_Fx, sum_Fy, sum_Fz, sum_Tx, sum_Ty, sum_Tz;
	while(i < INIT_SAMPLES) 
	{
		ros::spinOnce();
		sum_Fx.push_back(filtered_forces[0]);
		sum_Fy.push_back(filtered_forces[1]);
		sum_Fz.push_back(filtered_forces[2]);
		sum_Tx.push_back(filtered_torques[0]);
		sum_Ty.push_back(filtered_torques[1]);
		sum_Tz.push_back(filtered_torques[2]);
		i++;	
	}

	// Calculate biases
	bias_forces[0] = std::accumulate(sum_Fx.begin(), sum_Fx.end(), 0.0) / sum_Fx.size();
	bias_forces[1] = std::accumulate(sum_Fy.begin(), sum_Fy.end(), 0.0) / sum_Fy.size();
	bias_forces[2] = std::accumulate(sum_Fz.begin(), sum_Fz.end(), 0.0) / sum_Fz.size();
	bias_torques[0] = std::accumulate(sum_Tx.begin(), sum_Tx.end(), 0.0) / sum_Tx.size();
	bias_torques[1] = std::accumulate(sum_Ty.begin(), sum_Ty.end(), 0.0) / sum_Ty.size();
	bias_torques[2] = std::accumulate(sum_Tz.begin(), sum_Tz.end(), 0.0) / sum_Tz.size();

	cout << "\033[32m\033[1mHWR: F/T biases computed! \033[0m" << endl;
	
	init_flag = true;
}

// Filtering force/torque sensor measurements
void FilterFTSensor()
{
	int ii, jj;		
	// FIR: Moving Average Filtering
	if(filter_flag)
	{
		for(ii = 0; ii < N; ii++)
		{
			ros::spinOnce();
			// Fill the first N elements for the 3 forces and 3 torques
			for(jj = 0; jj < 3; jj++)
			{
				forces_cum[jj][ii]  = raw_forces[jj];
				torques_cum[jj][ii] = raw_torques[jj];
				sum_forces[jj]     += forces_cum[jj][ii];
				sum_torques[jj]    += torques_cum[jj][ii];
			}
		}
		filter_flag = false;
	}
	else
	{
		// Shift the moving average window
		for(ii = 0; ii < N-1; ii++)
		{
			for(jj = 0; jj < 3; jj++)
			{
				forces_cum[jj][ii]  = forces_cum[jj][ii+1];
				torques_cum[jj][ii] = torques_cum[jj][ii+1];
			}
		}
			
		ros::spinOnce();

		for(jj = 0; jj < 3; jj++)
		{
			// Add a new sample
			forces_cum[jj][N-1]  = raw_forces[jj];
			torques_cum[jj][N-1] = raw_torques[jj];
			// Update the sum
			sum_forces[jj]      += forces_cum[jj][N-1];
			sum_torques[jj]     += torques_cum[jj][N-1];
		}
	}

	// Compute filtered data (mean over N elements - biases)
	filtered_forces[0]  = (sum_forces[0]  / N) - bias_forces[0];
	filtered_forces[1]  = (sum_forces[1]  / N) - bias_forces[1];
	filtered_forces[2]  = (sum_forces[2]  / N) - bias_forces[2];
	filtered_torques[0] = (sum_torques[0] / N) - bias_torques[0];
	filtered_torques[1] = (sum_torques[1] / N) - bias_torques[1];
	filtered_torques[2] = (sum_torques[2] / N) - bias_torques[2];

	// As the window will be shifted at the next iteration, remove the first elements from the sums
	for(jj = 0; jj < 3; jj++)
	{
		sum_forces[jj]  -= forces_cum[jj][0];
		sum_torques[jj] -= torques_cum[jj][0];
	}

	// Compute force magnitude
	force_magnitude = sqrt(filtered_forces[0]*filtered_forces[0] + 
			       filtered_forces[1]*filtered_forces[1] + 
			       filtered_forces[2]*filtered_forces[2]);

	// Direction of the force expressed with polar and azimuth angles, which are then used into the HWR
	polar_angle = acos(filtered_forces[2]/force_magnitude);
	azimuth_angle = atan2(filtered_forces[1], filtered_forces[0]);

	// Publish F/T measurements for logging (with bias removal)
	log_ft_msg.force.x  = raw_forces[0]  - bias_forces[0];
	log_ft_msg.force.y  = raw_forces[1]  - bias_forces[1];
	log_ft_msg.force.z  = raw_forces[2]  - bias_forces[2];
	log_ft_msg.torque.x = raw_torques[0] - bias_torques[0];
	log_ft_msg.torque.y = raw_torques[1] - bias_torques[1];
	log_ft_msg.torque.z = raw_torques[2] - bias_torques[2];
	log_ft_pub.publish(log_ft_msg);

	// Publish filtered data
	ft_filtered_msg.force.x  = filtered_forces[0];
	ft_filtered_msg.force.y  = filtered_forces[1]; 
	ft_filtered_msg.force.z  = filtered_forces[2]; 
	ft_filtered_msg.torque.x = filtered_torques[0];
	ft_filtered_msg.torque.y = filtered_torques[1];
	ft_filtered_msg.torque.z = filtered_torques[2];
	ft_filtered_pub.publish(ft_filtered_msg);
}

void PrintStrategyInfo()
{
	cout << "\r\n\n\033[32m\033[1m--------------------------------------------------------------\033[0m" << endl;

	// Print filtered force/torque information
	cout << "Lateral Force (x) [N]: "  << filtered_forces[0]  << endl;
	cout << "Lateral Force (y) [N]: "  << filtered_forces[1]  << endl;
	cout << "Vertical Force (z) [N]: " << filtered_forces[2]  << endl;
	cout << "Force magnitude [N]: "    << force_magnitude     << endl;
	cout << "--" 		                                  << endl;
	cout << "Roll moment (x) [Nm] "    << filtered_torques[0] << endl;
	cout << "Pitch moment (y) [Nm] "   << filtered_torques[1] << endl;
	cout << "Yaw moment (z) [Nm] "     << filtered_torques[2] << endl;
	cout << "--" 		                                  << endl;

	cout << "Polar Angle: "   << (180/M_PI)*polar_angle   << " [deg], " << polar_angle   << " [rad]" << endl;	  
	cout << "Azimuth Angle: " << (180/M_PI)*azimuth_angle << " [deg], " << azimuth_angle << " [rad]" << endl;	 
	
	cout << "-------" << endl;

	switch(state)
	{		
		case 1: 
			cout << "\033[32m\033[1mLANDING PHASE! \033[0m" << endl;
			break;
		case 2: 
			cout << "\033[32m\033[1mCONTACT DETECTED: MINIMIZING ENERGY! \033[0m" << endl;
			break;
		case 3:
			cout << "\033[32m\033[1mRESTING PHASE! \033[0m" << endl;
			break;
		case 4:
			cout << "\033[32m\033[1mCOMING HOME! \033[0m" << endl;
			break;
		case 5:
                        cout << "\033[32m\033[1mMISSION ACCOMPLISHED! \033[0m" << endl;
                        break;
		default:
			break;
	}

	cout << "-------" << endl;

	// Print control information
	cout << "COMMANDED POLAR: "   << (180/M_PI)*commanded_polar   << " [deg], " << commanded_polar   << " [rad]" << endl;
	cout << "COMMANDED AZIMUTH: " << (180/M_PI)*commanded_azimuth << " [deg], " << commanded_azimuth << " [rad]" << endl;
	cout << "LAST EFFORT: "       << last_effort		      << endl;
	cout << "EFFORT: "            << pushing_effort               << endl;
			
	cout << "-------" << endl;

	// Print current position and commanded waypoint	
	cout << "Current waypoint        = X: "         << position[0] 
             <<                         ", Y: "         << position[1] 
             <<                         ", Z: "         << position[2] 
	     <<			        ", Yaw [deg]: " << (180/M_PI)*yaw
	                                                << endl;
	cout << "Publishing new waypoint = X: "         << ref_pose_msg.pose.position.x 
             <<                         ", Y: "         << ref_pose_msg.pose.position.y 
             <<                         ", Z: "         << ref_pose_msg.pose.position.z 
	     <<			        ", Yaw [deg]: " << (180/M_PI)*control_yaw
	                                                << endl;

	cout << "-------"  << endl;
	
	if(contact_flag)
	{
		cout << "Lateral slide (frontal, X): " << 100*(fabs(interaction_position[0] - position[0])) << " [cm]" << endl;
		cout << "Lateral slide (lateral, Y): " << 100*(fabs(interaction_position[1] - position[1])) << " [cm]" << endl;
	}	
}

void publishLog()
{	
	// Reference pose
	ref_pose_log_msg.position.x    = ref_position[0];
	ref_pose_log_msg.position.y    = ref_position[1];
	ref_pose_log_msg.position.z    = ref_position[2];
	ref_pose_log_msg.orientation.x = ref_orientation[0];
	ref_pose_log_msg.orientation.y = ref_orientation[1];
	ref_pose_log_msg.orientation.z = ref_orientation[2];
	ref_pose_log_msg.orientation.w = ref_orientation[3];
	ref_pose_log_pub.publish(ref_pose_log_msg);

	// State
	state_msg.data = state;
	state_pub.publish(state_msg);
}

// Send (publish) reference pose commands to the position controller
void SendReferencePose(double x_ref, double y_ref, double z_ref, double yaw_ref)
{
	// Reference postion: x, y, z computed by the HWR
	//ref_pose_msg.header.stamp = ros::Time::now();	
	ref_pose_msg.pose.position.x = x_ref;
	ref_pose_msg.pose.position.y = y_ref;
	ref_pose_msg.pose.position.z = z_ref;

	// Reference orientation: yaw -> quaternion 
	ref_pose_msg.pose.orientation.x = 0.0;	
	ref_pose_msg.pose.orientation.y = 0.0;	
	ref_pose_msg.pose.orientation.z = sin(yaw_ref/2);	
	ref_pose_msg.pose.orientation.w = cos(yaw_ref/2);

	ref_pose_pub.publish(ref_pose_msg);
}

// Haptic-based Waypoint Replanner (HWR) 
void ControlStrategy(Eigen::Vector3f ref_pos, double effort, double theta, double phi, double psi)
{
	// Saturation of the commanded polar angle
	if(theta > M_PI)
		theta = M_PI;
	else if(theta <= -M_PI)
		theta = -M_PI;

	// Staturation of the pushing effort
	if(effort >= MAX_EFFORT)
		effort = MAX_EFFORT;

	// New waypoint in the opposite direction of the forcen
	SendReferencePose(ref_pos[0] - (effort * control_gain_x * sin(theta) * cos(phi)), 
			  ref_pos[1] - (effort * control_gain_y * sin(theta) * sin(phi)),
			  ref_pos[2] - (effort * control_gain_z * cos(theta)),
			  psi);

	// Update control parameters
	commanded_polar   = theta;
	commanded_azimuth = phi;
	pushing_effort    = effort;
}

// State Machine handler
void StateMachine()
{
	if(!sliding_flag)
	{
		// No contact detected: drone in descedning phase (flying down)
		if(fabs(filtered_forces[2]) < MIN_VERTICAL_FORCE_THRESHOLD && !contact_flag)
		{
			state = 1;

			// First time landing
			if(!landing_flag)
			{
				// Save current position (useful for descending)
				landing_position = position;
				landing_flag = true;
				publishing_time  = ros::Time::now().toSec();
			}

			if((ros::Time::now().toSec() - publishing_time) > PUB_TIME) 
			{
				// Set control variables: lateral position is not changing, vertical is set to the current height
				pushing_effort = 1;	
				control_position[0] = landing_position[0];
				control_position[1] = landing_position[1];
				control_position[2] = position[2];
				
				control_yaw = initial_yaw;

				// Height reference decreased of a fixed amount related to pushing effort
				ControlStrategy(control_position, landing_gain*pushing_effort, 0.0, 0.0, control_yaw);

				PrintStrategyInfo();
				// Update
				publishing_time = ros::Time::now().toSec();			
			}
			
			// Set other variables
			sliding_flag   = false;
			contact_flag   = false;	
		}
		// Contact detected: Lean on the osbtacle until a threshold in lateral slippage or max force is overcome
		else if((fabs(filtered_forces[2]) > MIN_VERTICAL_FORCE_THRESHOLD && fabs(filtered_forces[2]) < MAX_VERTICAL_FORCE_THRESHOLD) || state == 2)
		{
			state = 2;

			// First time in contact
			if(!contact_flag)
			{
				// Save current position (useful for lateral slide)
				interaction_position = position;
				
				interaction_yaw = initial_yaw;	
				contact_flag = true;
				pushing_effort = 1;
			}

			// Increase the pushing effort every PUB_TIME seconds to make the drone push more and more down
			if((ros::Time::now().toSec() - publishing_time) > PUB_TIME) 
			{
				// Set variables: lateral position as the first point of contact (fixed), vertical as the current height (variable)
				control_position[0] = interaction_position[0];
				control_position[1] = interaction_position[1];
				control_position[2] = position[2];

				control_yaw = interaction_yaw;
		
				// Update the pushing effort
				pushing_effort += DELTA_PUSH;
				// Height decreases in accord to the pushing effort to lay on the detected obstacle
				ControlStrategy(control_position, pushing_effort, 0.0, 0.0, control_yaw);

				PrintStrategyInfo();
				// Update
				publishing_time = ros::Time::now().toSec();
			}
			
			sliding_flag = false;

			// Compute lateral slide on x and y
			lateral_slide_x = fabs(interaction_position[0] - position[0]);
			lateral_slide_y = fabs(interaction_position[1] - position[1]);
		}
		last_effort = pushing_effort;
	}
	else
	{
		if((ros::Time::now().toSec() - publishing_time) > PUB_TIME) 
		{
			// Print in the terminal
			PrintStrategyInfo();
			// Update
			publishing_time = ros::Time::now().toSec();
		}
		if((state == 3) && ((ros::Time::now().toSec() - resting_time) > RESTING_TIME)) 
		{
			state = 4;
			steps = 1;
			// Start the timer for the homing phase
                	resting_time = ros::Time::now().toSec();
		}
	}

	// Max threshold overcome OR the drone is sliding: Resting
	if((max(lateral_slide_x,lateral_slide_y) > MAX_LATERAL_SLIDING || fabs(filtered_forces[2]) > MAX_VERTICAL_FORCE_THRESHOLD) && contact_flag && !sliding_flag)
	{
		state = 3;

		// Update variable
		sliding_flag = true;	// Therefore waypoint computed only once
		control_position = position;
		control_yaw = initial_yaw;

		ControlStrategy(control_position, last_effort, polar_angle, azimuth_angle, control_yaw); 

		// Start the timer for the resting phase
		resting_time = ros::Time::now().toSec();
	}	

	// After the resting phase, go home
	if(state == 4)
	{
		if((ros::Time::now().toSec() - homing_time) > HOME_TIME)
                {
                        PrintStrategyInfo();
                    
			if(steps == 1)
			{
				// Go up, 0.25cm above the resting spot
				control_yaw = initial_yaw;
				SendReferencePose(interaction_position[0], interaction_position[1], position[2] + 0.25, control_yaw);
			}
			else if(steps == 2)
			{
				if(!outdoor_flag)
				{
					// If you are indoor go back to x=0.0, y=0.0
					control_yaw = 0.0;
					SendReferencePose(0.0, 0.0, position[2], control_yaw);
			
				}
				else
					// If you are outdoor keep hovering above the branch (exit the external if)
					state = 5;
			}
			else if(steps == 3)
			{
				// This engages only if you are indoor, you also go back to z=0.0 (base)
				SendReferencePose(0.0, 0.0, 0.0, control_yaw);
				state = 5;
			}
			// Update
                        homing_time = ros::Time::now().toSec();
			steps++;
		}	
	}
}

//////////////////////////////////////// Main ////////////////////////////////////////

// Main function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "haptic_waypoint_replanner");
	ros::NodeHandle nh("~");
	
	// Set gains
	if(nh.getParam("gain_x", control_gain_x))
		cout << "Control Gain X found: " << control_gain_x << endl;
	else
	{
		cout << "Control Gain X NOT found! Set up as default value" << endl;
		control_gain_x = 0.001;
	}
	
	if(nh.getParam("gain_y", control_gain_y))
		cout << "Control Gain Y found: " << control_gain_y << endl;
	else
	{
		cout << "Control Gain Y NOT found! Set up as default value" << endl;
		control_gain_y = 0.001;
	}

	if(nh.getParam("gain_z", control_gain_z))
		cout << "Control Gain Z found: " << control_gain_z << endl;
	else
	{
		cout << "Control Gain Z NOT found! Set up as default value" << endl;
		control_gain_z = 0.01;
	}

	if(nh.getParam("min_vertical_force_threshold", MIN_VERTICAL_FORCE_THRESHOLD))
		cout << "MIN_VERTICAL_FORCE_THRESHOLD found: " << MIN_VERTICAL_FORCE_THRESHOLD << endl;
	else
	{
		cout << "MIN_VERTICAL_FORCE_THRESHOLD NOT found! Set up as default value" << endl;
		MIN_VERTICAL_FORCE_THRESHOLD = 0.35;
	}

	if(nh.getParam("max_vertical_force_threshold", MAX_VERTICAL_FORCE_THRESHOLD))
		cout << "MAX_VERTICAL_FORCE_THRESHOLD found: " << MAX_VERTICAL_FORCE_THRESHOLD << endl;
	else
	{
		cout << "MAX_VERTICAL_FORCE_THRESHOLD NOT found! Set up as default value" << endl;
		MAX_VERTICAL_FORCE_THRESHOLD = 1.0;
	}

	if(nh.getParam("pushing_effort", pushing_effort))
		cout << "pushing_effort found: " << pushing_effort << endl;
	else
	{
		cout << "pushing_effort NOT found! Set up as default value" << endl;
		pushing_effort = 1;
	}
	
	if(nh.getParam("landing_gain", landing_gain))
		cout << "landing_gain found: " << landing_gain << endl;
	else
	{
		cout << "landing_gain NOT found! Set up as default value" << endl;
		landing_gain = 5;
	}

	if(nh.getParam("delta_pushing_effort", DELTA_PUSH))
		cout << "DELTA_PUSH found: " << DELTA_PUSH << endl;
	else
	{
		cout << "DELTA_PUSH NOT found! Set up as default value" << endl;
		DELTA_PUSH = 1;
	}
	
	if(nh.getParam("resting_time", RESTING_TIME))
		cout << "RESTING_TIME found: " << RESTING_TIME << endl;
	else
	{
		cout << "RESTING_TIME NOT found! Set up as default value" << endl;
		RESTING_TIME = 10;
	}
	if(nh.getParam("outdoor", outdoor))
	{
		if(outdoor == 1)
                        outdoor_flag = true;
                else
                        outdoor_flag = false;
                cout << "outdoor_flag found: " << outdoor_flag << endl;
	}
	else
        {
                cout << "outdoor_flag NOT found! Set up as default value" << endl;
                outdoor_flag = false;
        }
	
	ros::Rate loop(100);

	start_cmd_sub        = nh.subscribe("/khadas_drone/haptic_waypoint_replanner/start", 10, &startCallback);
	ft_sensor_sub        = nh.subscribe("/rokubimini/ft_sensor0/ft_sensor_readings/wrench", 10, &forceTorqueSensorCallback);
	odom_sub 	     = nh.subscribe("/khadas_drone/state_estimate", 10, &odometryCallback);
	ref_pose_sub         = nh.subscribe("/khadas_drone/autopilot/pose_command", 10, &refPoseCallback);
	ref_pose_pub     = nh.advertise<geometry_msgs::PoseStamped>("/khadas_drone/autopilot/pose_command", 10);
	ref_pose_log_pub     = nh.advertise<geometry_msgs::Pose>("/khadas_drone/reference_pose_log", 10);
	log_ft_pub 	     = nh.advertise<geometry_msgs::Wrench>("/khadas_drone/haptic_waypoint_replanner/ft_log", 10);
	ft_filtered_pub      = nh.advertise<geometry_msgs::Wrench>("/khadas_drone/haptic_waypoint_replanner/ft_filtered", 10);
	state_pub	     = nh.advertise<std_msgs::Int32>("/khadas_drone/haptic_waypoint_replanner/state", 10);

	ref_pose_log_msg.position.x = 0.0;
	ref_pose_log_msg.position.y = 0.0;
	ref_pose_log_msg.position.z = 0.0;
	ref_pose_log_msg.orientation.x = 0.0;
	ref_pose_log_msg.orientation.y = 0.0;
	ref_pose_log_msg.orientation.z = 0.0;
	ref_pose_log_msg.orientation.w = 1.0;

	while(ros::ok())
	{	
		ros::spinOnce();

		// Filter force/torque measurements
		FilterFTSensor();
		
		if(start_flag)	// when the start command is received
		{
			if(!init_flag)	// inizialize load cell biases
			{
				InitializeBiases();
				publishing_time  = ros::Time::now().toSec();
			}
			else	// when the biases are initialized
			{
				if(RESTING_TIME > 0)
					StateMachine();
			}
		}

		// Publish useful data
		publishLog();
	
		loop.sleep();       
	}

	return 0;
}


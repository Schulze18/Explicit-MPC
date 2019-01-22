//This plugin is based on the tutorial to control a Velodyne sensor

#ifndef _IRIS_PLUGIN_HH_
#define _IRIS_PLUGIN_HH_

//Include default libraries for Gazebo and ROS interface
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
//Include Msgs Types
#include <sensor_msgs/Imu.h> 
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>


namespace gazebo
{
  //A plugin to control a Velodyne sensor.
  class IrisPlugin : public ModelPlugin
  {
    //brief Constructor
    public: IrisPlugin() {}

    //load function is called by Gazebo when the plugin isinserted into simulation
    // \param[in] _model A pointer to the model that this plugin is attached to.
    // \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

    // Just output a message for now
    std::cerr << "\nThe iris plugin is attach to model[" << _model->GetName() << "\n";

	// Store the model pointer for convenience.
	this->model = _model;	

	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
	{
	  int argc = 0;
	  char **argv = NULL;
	  ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

	// Create and subscribe to a topic with Quaternion type
	ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Quaternion>(
	"/" + this->model->GetName() + "/vel_cmd",1,
	boost::bind(&IrisPlugin::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);

	// Store the subscriber for convenience.
	this->rosSub = this->rosNode->subscribe(so);

	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&IrisPlugin::QueueThread, this));

	// Create a topic to publish iris state
	this->state_pub = this->rosNode->advertise<sensor_msgs::Imu>("iris_state", 100);
	
	// Configure Timer and callback function
	this->pubTimer = this->rosNode->createTimer(ros::Duration(0.01), &IrisPlugin::pub_callback,this);
	}

	// Update the velocity applied to the Rotors
	//param[in] vel is an array with the desired velocities
	public: void UpdateVelocity(double vel[4])
    {
	double k = 0.00000298;
	double b = 0.000000114;	

	math::Vector3 thrust_force;
	
	thrust_force.Set(0,0,k*vel[0]*vel[0]);		
	this->model->GetLinks()[3]->AddRelativeForce(thrust_force);

	thrust_force.Set(0,0,k*vel[1]*vel[1]);	
	this->model->GetLinks()[4]->AddRelativeForce(thrust_force);

	thrust_force.Set(0,0,k*vel[2]*vel[2]);	
	this->model->GetLinks()[5]->AddRelativeForce(thrust_force);

	thrust_force.Set(0,0,k*vel[3]*vel[3]);	
	this->model->GetLinks()[6]->AddRelativeForce(thrust_force);

    }


	// Handle an incoming message from ROS
	// param[in] _msg A float value that is used to set the velocity of the Iris Rotors
	public: void OnRosMsg(const geometry_msgs::QuaternionConstPtr& msg)
	{

	double test[4] = { msg->x, msg->y, msg->z, msg->w};

	this->UpdateVelocity(test);
	}

	// ROS helper function that processes messages
	private: void QueueThread()
	{
	  static const double timeout = 0.01;
	  while (this->rosNode->ok())
	  {
	    this->rosQueue.callAvailable(ros::WallDuration(timeout));
	  }
	}


	public: void pub_callback(const ros::TimerEvent& event)
	{

	//this->model->GetWorldPose(); ---- Return the linear and angular position of Iris 
			//this->model->GetWorldPose().pos
			//this->model->GetWorldPose().rot.GetPitch();
			//this->model->GetWorldPose().rot.GetRoll();
			//this->model->GetWorldPose().rot.GetYaw();
	//this->model->GetWorldLinearVel(); ---- Return the linear velocity (xp, yp,zp)
	//this->model->GetWorldAngularVel(); ---- Return the angular velocity 

	math::Pose iris_pose;
	math::Vector3 iris_linear_vel, iris_angular_vel;

	iris_pose = this->model->GetWorldPose();
	iris_linear_vel = this->model->GetWorldLinearVel();
	iris_angular_vel = this->model->GetWorldAngularVel();
	

	// Create a variable with the same type of the topic
	sensor_msgs::Imu iris_state;

	// Store the data in the right type for publish
	iris_state.orientation_covariance[0] = iris_pose.pos[0];
	iris_state.orientation_covariance[1] = iris_pose.pos[1];
	iris_state.orientation_covariance[2] = iris_pose.pos[2];

	iris_state.orientation_covariance[3] = iris_pose.rot.GetPitch();
	iris_state.orientation_covariance[4] = iris_pose.rot.GetRoll();
	iris_state.orientation_covariance[5] = iris_pose.rot.GetYaw();
	
	iris_state.angular_velocity_covariance[0] = iris_linear_vel[0];
	iris_state.angular_velocity_covariance[1] = iris_linear_vel[1];
	iris_state.angular_velocity_covariance[2] = iris_linear_vel[2];
	
	iris_state.angular_velocity_covariance[3] = iris_angular_vel[0];
	iris_state.angular_velocity_covariance[4] = iris_angular_vel[1];
	iris_state.angular_velocity_covariance[5] = iris_angular_vel[2];

	// Publish iris_state
	this->state_pub.publish(iris_state);

	// Print the time just for Debug
	ros::Time begin = ros::Time::now();
	std::cout << begin << "\n";

	}
	
	// A node used for transport
	private: transport::NodePtr node;

	// A subscriber to a named topic.
	private: transport::SubscriberPtr sub;

	// Pointer to the model.
	private: physics::ModelPtr model;

	// Pointer to the joint.
	private: physics::JointPtr joint;

	// A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;
	
	// A ROS subscriber
	private: ros::Subscriber rosSub;

	// Publisher of Iris States
	private: ros::Publisher state_pub; 

	// A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;

	// A thread the keeps running the rosQueue
	private: std::thread rosQueueThread;  

	// Publisher Timer
	private: ros::Timer pubTimer;

	};

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(IrisPlugin)
}
#endif

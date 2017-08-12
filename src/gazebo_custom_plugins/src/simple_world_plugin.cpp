#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
class ModelPluginTutorial : public ModelPlugin
{
public:
  ModelPluginTutorial()// : ModelPlugin()
  {
  }

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the model pointer for convenience.
    this->model = _model;

    // Get the first joint. We are making an assumption about the model
    // having one joint that is the rotational joint.
    this->joint = _model->GetJoints()[0];

    // Setup a P-controller, with a gain of 0.1.
    this->pid = common::PID(1.0, 1.0, 0);

    /*// Apply the P-controller to the joint.
    this->model->GetJointController()->SetVelocityPID(
        this->joint->GetScopedName(), this->pid);

    // Default to zero velocity
    double velocity = 10.0;

    // Check that the velocity element exists, then read the value
    if (_sdf->HasElement("velocity"))
      velocity = _sdf->Get<double>("velocity");

    this->SetVelocity(velocity);

    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());

    // Create a topic name
    std::string topicName = "/walao";

    // Subscribe to the topic, and register a callback
    this->sub = this->node->Subscribe(topicName,
       &ModelPluginTutorial::OnMsg, this);

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
          ros::init_options::NoSigintHandler);
    }

    ROS_INFO("%s \n\n\n\n\n", _model->GetJoints()[0]->GetName().c_str());
    ROS_INFO("%s \n\n\n\n\n\n\n\n", this->joint->GetScopedName().c_str());
    ROS_INFO("%f \n\n\n\n\n\n\n\n", velocity);
    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Float32>(
          "/" + this->model->GetName() + "/walao",
          1,
          boost::bind(&ModelPluginTutorial::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
      std::thread(std::bind(&ModelPluginTutorial::QueueThread, this));*/
  }

  /// \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
  public: void SetVelocity(const double &_vel)
  {
    // Set the joint's target velocity.
    this->model->GetJointController()->SetVelocityTarget(
        this->joint->GetScopedName(), _vel);
  }

  /// \brief Handle incoming message
  /// \param[in] _msg Repurpose a vector3 message. This function will
  /// only use the x component.
  private: void OnMsg(ConstVector3dPtr &_msg)
  {
    this->SetVelocity(_msg->x());
  }

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the Velodyne.
  public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
  {
    this->SetVelocity(_msg->data);
  }

  /// \brief ROS helper function that processes messages
  private: void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  /// \brief A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
  private: ros::Subscriber rosSub;

  /// \brief A ROS callbackqueue that helps process messages
  private: ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  private: std::thread rosQueueThread;

  /// \brief A node used for transport
  private: transport::NodePtr node;

  /// \brief A subscriber to a named topic.
  private: transport::SubscriberPtr sub;

  /// \brief Pointer to the model.
  private: physics::ModelPtr model;

  /// \brief Pointer to the joint.
  private: physics::JointPtr joint;

  /// \brief A PID controller for the joint.
  private: common::PID pid;

};
GZ_REGISTER_MODEL_PLUGIN(ModelPluginTutorial)
}

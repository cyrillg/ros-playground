#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>

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
    ROS_INFO("\n\n\n\n\n %s \n\n\n\n\n", _model->GetJoints()[0]->GetName().c_str());
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    // Store the model pointer for convenience.
    this->model = _model;

    // Get the first joint. We are making an assumption about the model
    // having one joint that is the rotational joint.
    this->joint = _model->GetJoints()[0];

    // Default to zero velocity
    double velocity = 10.0;

    ROS_INFO("\n\n\n\n\n\n\n\n %s \n\n\n\n\n\n\n\n", this->joint->GetScopedName().c_str());
    ROS_INFO("\n\n\n\n\n\n\n\n %f \n\n\n\n\n\n\n\n", velocity);
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
  }

  /// \brief Set the velocity of the Velodyne
  /// \param[in] _vel New target velocity
  public: void SetVelocity(const double &_vel)
  {
    // Set the joint's target velocity.
    this->model->GetJointController()->SetVelocityTarget(
        this->joint->GetScopedName(), _vel);
  }

  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  this->joint = _model->GetJoints()[0];

  /// \brief Handle incoming message
  /// \param[in] _msg Repurpose a vector3 message. This function will
  /// only use the x component.
  private: void OnMsg(ConstVector3dPtr &_msg)
  {
    this->SetVelocity(_msg->x());
  }

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

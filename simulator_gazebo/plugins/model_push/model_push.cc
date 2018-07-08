#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: ModelPush() : ModelPlugin()
    {
      printf("Loaded target push plugin. \n");
      this->topicName = "/launcher_gazebo/set_model_dynamics";
      this->vel.x     = 0;
      this->vel.y     = 0;
      this->vel.z     = 0;
    }
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      // get name of the target model
      this->currentModel = this->model->GetName();

      if(!ros::isInitialized())
      {
        int argc    = 0;
        char **argv = NULL;
        ros::init(argc, argv, "modelDynamics");
      }
      this->rosNode.reset(new ros::NodeHandle("modelDynamics"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::string>(
            this->topicName,
            1,
            boost::bind(&ModelPush::OnMsg, this, _1), //equivalent to ModelPush::OnMsg(this, ros_data)
            ros::VoidPtr(),
            &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      // Spin up the queue helper thread. equivalent to rospy.spin()
      this->rosQueueThread = boost::thread(std::bind(&ModelPush::QueueThread, this));

      // Apply a small linear velocity to the model.
      this->link = this->model->GetChildLink("link");
    }
    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(this->vel.x, this->vel.y, this->vel.z));

    }
    // Called by publishing a message on the corresponding ROS topic
    public: void OnMsg(const detect_and_hit::TargetSpawnConstPtr &_ros_data)
    {
      // all targets have the same topic they are subscribing to;
      // in order to set the velocity only to the current model, we created a message type which has
      // the name of the model included;
    // equivalent to rospy.spin()
      if(this->currentModel.compare(_ros_data->targetName.data) == 0)
      {
        this->vel.x = _ros_data->vel.x;
        this->vel.y = _ros_data->vel.y;
        this->vel.z = _ros_data->vel.z;
      }
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the link
    private: physics::LinkPtr link;
    // ROS node
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: boost::thread rosQueueThread;

    // topic name to which this node subscribes
    private: std::string topicName;

    //
    private: std::string stopThread;

    // current model name
    private: std::string currentModel;
    //
    private: geometry_msgs::Point vel;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)

}

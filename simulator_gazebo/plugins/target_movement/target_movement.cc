#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/advertise_service_options.h"
#include "ros/node_handle.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Point.h"
#include <simulator_gazebo/SetVelocity.h>

namespace gazebo
{
  class TargetMovement : public ModelPlugin
  {
    public: TargetMovement() : ModelPlugin()
    {
      printf("Loaded target movement plugin. \n");
      this->vel.x = 0;
      this->vel.y = 0;
      this->vel.z = 0;
      this->rosSrvName = "";
      this->done = false;
    }
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      // get name of the target model
      this->currentModel = this->model->GetName();

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TargetMovement::OnUpdate, this));

      this->link = this->model->GetChildLink("link");

      // start ROS node
      int argc    = 0;
      char **argv = NULL;
      ros::init(argc, argv, "set_target_velocity");
      //ros::ServiceServer service = this->rosNode.advertiseService("set_velocity", &TargetMovement::ServerCallback, this);


      // create service name for each model by concatenating current model name and "/set_velocity"
      this->rosSrvName = this->currentModel.append("/set_velocity");
      // create service server
      this->servOpt = ros::AdvertiseServiceOptions::create<simulator_gazebo::SetVelocity>
                (
                  this->rosSrvName,
                  boost::bind(&TargetMovement::ServerCallback,this,_1,_2),
                  ros::VoidPtr(),
                  &this->rosQueue
                );
      this->rosSrv = this->rosNode.advertiseService(this->servOpt);

      // thread for dealing with service calls
      this->rosQueueThread = std::thread(std::bind(&TargetMovement::QueueThread,this));

    }
    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3d(this->vel.x, this->vel.y, this->vel.z));

    }
    // called by a request from a client
    public: bool ServerCallback(simulator_gazebo::SetVelocityRequest &request, simulator_gazebo::SetVelocityResponse &response)
    {
        printf("working");
        this->vel.x = request.vel.x;
        this->vel.y = request.vel.y;
        this->vel.z = request.vel.z;
        response.done = true;
        this->done = true;
        return true;
    }

    public: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode.ok())
      {
        if(this->done)
        {
          // detach thread for safe delete of the model; break the loop
          this->rosQueueThread.detach();
          break;
        }
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the link
    private: physics::LinkPtr link;
    // current model name
    private: std::string currentModel;
    //
    private: geometry_msgs::Point vel;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;


    // ros node and callback queue
    private: ros::NodeHandle rosNode;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;

    // advertise service options
    private: ros::AdvertiseServiceOptions servOpt;
    private: ros::ServiceServer rosSrv;

    // ros service name
    private: std::string rosSrvName;
    // after model gets velocity, done equals true and we can safely delete the model from the simulation
    private: bool done;

  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TargetMovement)

}

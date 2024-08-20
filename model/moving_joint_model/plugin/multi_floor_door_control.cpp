#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <iostream>

namespace gazebo
{
  class MultiFloorDoorControl : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;

      // Get joints
      this->joint1 = this->model->GetJoint("joint1");
      this->joint2 = this->model->GetJoint("joint2");
      this->joint3 = this->model->GetJoint("joint3");
      this->joint4 = this->model->GetJoint("joint4");

      if (!this->joint1 || !this->joint2 || !this->joint3 || !this->joint4)
      {
        gzerr << "One or more joints not found!" << std::endl;
        return;
      }

      // Initialize Gazebo transport
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      // Subscribe to the ~/elevator topic
      this->sub = this->node->Subscribe("~/elevator", &MultiFloorDoorControl::OnMsg, this);

      std::cout << "MultiFloorDoorControl loaded successfully!" << std::endl;
    }

    void OnMsg(ConstIntPtr &msg)
    {
      int command = msg->data();

      switch (command)
      {
        case 0:
          if (this->joint1) 
            this->joint1->SetPosition(0, 1.0);  // Activate joint1
          if (this->joint2) 
            this->joint2->SetPosition(0, 0.0);  // Deactivate joint2
          if (this->joint3) 
            this->joint3->SetPosition(0, 0.0);  // Deactivate joint3
          if (this->joint4) 
            this->joint4->SetPosition(0, 0.0);  // Deactivate joint4
          break;
        
        case 1:
          if (this->joint1) 
            this->joint1->SetPosition(0, 0.0);  // Deactivate joint1
          if (this->joint2) 
            this->joint2->SetPosition(0, 1.0);  // Activate joint2
          if (this->joint3) 
            this->joint3->SetPosition(0, 0.0);  // Deactivate joint3
          if (this->joint4) 
            this->joint4->SetPosition(0, 0.0);  // Deactivate joint4
          break;
        
        case 2:
          if (this->joint1) 
            this->joint1->SetPosition(0, 0.0);  // Deactivate joint1
          if (this->joint2) 
            this->joint2->SetPosition(0, 0.0);  // Deactivate joint2
          if (this->joint3) 
            this->joint3->SetPosition(0, 1.0);  // Activate joint3
          if (this->joint4) 
            this->joint4->SetPosition(0, 0.0);  // Deactivate joint4
          break;
        
        case 3:
          if (this->joint1) 
            this->joint1->SetPosition(0, 0.0);  // Deactivate joint1
          if (this->joint2) 
            this->joint2->SetPosition(0, 0.0);  // Deactivate joint2
          if (this->joint3) 
            this->joint3->SetPosition(0, 0.0);  // Deactivate joint3
          if (this->joint4) 
            this->joint4->SetPosition(0, 1.0);  // Activate joint4
          break;
        
        default:
          std::cerr << "Unknown command: " << command << std::endl;
          break;
      }
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr joint1;
    physics::JointPtr joint2;
    physics::JointPtr joint3;
    physics::JointPtr joint4;

    transport::NodePtr node;
    transport::SubscriberPtr sub;
  };

  GZ_REGISTER_MODEL_PLUGIN(MultiFloorDoorControl)
}

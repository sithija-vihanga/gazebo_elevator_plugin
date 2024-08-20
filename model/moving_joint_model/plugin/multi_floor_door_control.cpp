#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <iostream>
#include <thread>
#include <chrono>

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

      // Start a timer to update the control loop
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MultiFloorDoorControl::OnUpdate, this));
    }

    void OnMsg(ConstIntPtr &msg)
    {
      int command = msg->data();
      this->targetJoint = nullptr;  // Reset target joint
      double velocity = 0.3; // The velocity to set for the active joint
      double zero_velocity = 0.0;

      switch (command)
      {
        case 0:
          this->targetJoint = this->joint1;
          break;

        case 1:
          this->targetJoint = this->joint2;
          break;

        case 2:
          this->targetJoint = this->joint3;
          break;

        case 3:
          this->targetJoint = this->joint4;
          break;

        case 4:
          this->targetJoint = nullptr; // Stop all joints
          break;

        case 5:
          // Set reverse velocity for all joints
          this->SetJointVelocity(this->joint1, -velocity);
          this->SetJointVelocity(this->joint2, -velocity);
          this->SetJointVelocity(this->joint3, -velocity);
          this->SetJointVelocity(this->joint4, -velocity);
          break;

        default:
          std::cerr << "Unknown command: " << command << std::endl;
          break;
      }
    }

    void OnUpdate()
    {
      if (this->targetJoint)
      {
        // Set the desired position limit
        double desiredPosition = 0.8;

        // Check if the joint is moving toward the desired position
        double currentPosition = this->targetJoint->Position(0);
        if (currentPosition < desiredPosition)
        {
          // Move toward the desired position
          this->SetJointVelocity(this->targetJoint, 0.3); // Move joint positively
        }
        else
        {
          // Stop the joint once the desired position is reached
          this->SetJointVelocity(this->targetJoint, 0.0);
          std::cout << "Joint reached target position!" << std::endl;
        }
      }
      else
      {
        // Stop all joints if no target joint is set
        this->SetJointVelocity(this->joint1, 0.0);
        this->SetJointVelocity(this->joint2, 0.0);
        this->SetJointVelocity(this->joint3, 0.0);
        this->SetJointVelocity(this->joint4, 0.0);
      }
    }

  private:
    void SetJointVelocity(physics::JointPtr joint, double velocity)
    {
      if (!joint) return;
      joint->SetVelocity(0, velocity); // Set the velocity of the joint
    }

    physics::ModelPtr model;
    physics::JointPtr joint1;
    physics::JointPtr joint2;
    physics::JointPtr joint3;
    physics::JointPtr joint4;
    physics::JointPtr targetJoint; // Pointer to the current target joint
    transport::NodePtr node;
    transport::SubscriberPtr sub;
    event::ConnectionPtr updateConnection; // Connection to the update event
  };

  GZ_REGISTER_MODEL_PLUGIN(MultiFloorDoorControl)
}

//version 2.0
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/Plugin.hh>
// #include <gazebo/msgs/msgs.hh>
// #include <gazebo/transport/transport.hh>
// #include <iostream>
// #include <thread>
// #include <chrono>

// namespace gazebo
// {
//   class MultiFloorDoorControl : public ModelPlugin
//   {
//   public:
//     void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
//     {
//       this->model = _model;

//       // Get joints
//       this->joint1 = this->model->GetJoint("joint1");
//       this->joint2 = this->model->GetJoint("joint2");
//       this->joint3 = this->model->GetJoint("joint3");
//       this->joint4 = this->model->GetJoint("joint4");

//       if (!this->joint1 || !this->joint2 || !this->joint3 || !this->joint4)
//       {
//         gzerr << "One or more joints not found!" << std::endl;
//         return;
//       }

//       // Initialize Gazebo transport
//       this->node = transport::NodePtr(new transport::Node());
//       this->node->Init();

//       // Subscribe to the ~/elevator topic
//       this->sub = this->node->Subscribe("~/elevator", &MultiFloorDoorControl::OnMsg, this);

//       std::cout << "MultiFloorDoorControl loaded successfully!" << std::endl;
//     }

//     void OnMsg(ConstIntPtr &msg)
//     {
//       int command = msg->data();
//       double velocity = 0.3; // The velocity to set for the active joint
//       double zero_velocity = 0.0;

//       switch (command)
//       {
//         case 0:
//           this->SetJointVelocity(this->joint1, velocity);
//           this->SetJointVelocity(this->joint2, zero_velocity);
//           this->SetJointVelocity(this->joint3, zero_velocity);
//           this->SetJointVelocity(this->joint4, zero_velocity);
//           this->SetLimits(this->joint1, 0.8);
//           break;
        
//         case 1:
//           this->SetJointVelocity(this->joint1, zero_velocity);
//           this->SetJointVelocity(this->joint2, velocity);
//           this->SetJointVelocity(this->joint3, zero_velocity);
//           this->SetJointVelocity(this->joint4, zero_velocity);
//           this->SetLimits(this->joint2, 0.8);
//           break;
        
//         case 2:
//           this->SetJointVelocity(this->joint1, zero_velocity);
//           this->SetJointVelocity(this->joint2, zero_velocity);
//           this->SetJointVelocity(this->joint3, velocity);
//           this->SetJointVelocity(this->joint4, zero_velocity);
//           this->SetLimits(this->joint3, 0.8);
//           break;
        
//         case 3:
//             this->SetJointVelocity(this->joint1, zero_velocity);
//             this->SetJointVelocity(this->joint2, zero_velocity);
//             this->SetJointVelocity(this->joint3, zero_velocity);
//             this->SetJointVelocity(this->joint4, velocity);
        
//         case 4:
//           this->SetJointVelocity(this->joint1, zero_velocity);
//           this->SetJointVelocity(this->joint2, zero_velocity);
//           this->SetJointVelocity(this->joint3, zero_velocity);
//           this->SetJointVelocity(this->joint4, zero_velocity);
//           break;

//         case 5:
//           this->SetJointVelocity(this->joint1, -velocity);
//           this->SetJointVelocity(this->joint2, -velocity);
//           this->SetJointVelocity(this->joint3, -velocity);
//           this->SetJointVelocity(this->joint4, -velocity);
//           break;
        
//         default:
//           std::cerr << "Unknown command: " << command << std::endl;
//           break;
//       }
//     }

//   private:
//     void SetJointVelocity(physics::JointPtr joint, double velocity)
//     {
//       if (!joint) return;

//       joint->SetVelocity(0, velocity); // Set the velocity of the joint
//     }

//   void SetLimits(physics::JointPtr joint, double position)
//   {
//       if (!joint) return;

//       while (joint->Position(0) < position)
//       {
//           std::cout << "Door opening!" << std::endl;
//           std::this_thread::sleep_for(std::chrono::milliseconds(100));
//       }
//       joint->SetVelocity(0, 0.0);
//       std::cout << "Door opened!" << std::endl;
//   }

//     physics::ModelPtr model;
//     physics::JointPtr joint1;
//     physics::JointPtr joint2;
//     physics::JointPtr joint3;
//     physics::JointPtr joint4;

//     transport::NodePtr node;
//     transport::SubscriberPtr sub;
//   };

//   GZ_REGISTER_MODEL_PLUGIN(MultiFloorDoorControl)
// }

// Version 1.0
// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/Plugin.hh>
// #include <gazebo/msgs/msgs.hh>
// #include <gazebo/transport/transport.hh>
// #include <iostream>

// namespace gazebo
// {
//   class MultiFloorDoorControl : public ModelPlugin
//   {
//   public:
//     void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
//     {
//       this->model = _model;

//       // Get joints
//       this->joint1 = this->model->GetJoint("joint1");
//       this->joint2 = this->model->GetJoint("joint2");
//       this->joint3 = this->model->GetJoint("joint3");
//       this->joint4 = this->model->GetJoint("joint4");

//       if (!this->joint1 || !this->joint2 || !this->joint3 || !this->joint4)
//       {
//         gzerr << "One or more joints not found!" << std::endl;
//         return;
//       }

//       // Initialize Gazebo transport
//       this->node = transport::NodePtr(new transport::Node());
//       this->node->Init();

//       // Subscribe to the ~/elevator topic
//       this->sub = this->node->Subscribe("~/elevator", &MultiFloorDoorControl::OnMsg, this);

//       std::cout << "MultiFloorDoorControl loaded successfully!" << std::endl;
//     }

//     void OnMsg(ConstIntPtr &msg)
//     {
//       int command = msg->data();

//       switch (command)
//       {
//         case 0:
//           if (this->joint1) 
//             this->joint1->SetPosition(0, 1.0);  // Activate joint1
//           if (this->joint2) 
//             this->joint2->SetPosition(0, 0.0);  // Deactivate joint2
//           if (this->joint3) 
//             this->joint3->SetPosition(0, 0.0);  // Deactivate joint3
//           if (this->joint4) 
//             this->joint4->SetPosition(0, 0.0);  // Deactivate joint4
//           break;
        
//         case 1:
//           if (this->joint1) 
//             this->joint1->SetPosition(0, 0.0);  // Deactivate joint1
//           if (this->joint2) 
//             this->joint2->SetPosition(0, 1.0);  // Activate joint2
//           if (this->joint3) 
//             this->joint3->SetPosition(0, 0.0);  // Deactivate joint3
//           if (this->joint4) 
//             this->joint4->SetPosition(0, 0.0);  // Deactivate joint4
//           break;
        
//         case 2:
//           if (this->joint1) 
//             this->joint1->SetPosition(0, 0.0);  // Deactivate joint1
//           if (this->joint2) 
//             this->joint2->SetPosition(0, 0.0);  // Deactivate joint2
//           if (this->joint3) 
//             this->joint3->SetPosition(0, 1.0);  // Activate joint3
//           if (this->joint4) 
//             this->joint4->SetPosition(0, 0.0);  // Deactivate joint4
//           break;
        
//         case 3:
//           if (this->joint1) 
//             this->joint1->SetPosition(0, 0.0);  // Deactivate joint1
//           if (this->joint2) 
//             this->joint2->SetPosition(0, 0.0);  // Deactivate joint2
//           if (this->joint3) 
//             this->joint3->SetPosition(0, 0.0);  // Deactivate joint3
//           if (this->joint4) 
//             this->joint4->SetPosition(0, 1.0);  // Activate joint4
//           break;
        
//         default:
//           std::cerr << "Unknown command: " << command << std::endl;
//           break;
//       }
//     }

//   private:
//     physics::ModelPtr model;
//     physics::JointPtr joint1;
//     physics::JointPtr joint2;
//     physics::JointPtr joint3;
//     physics::JointPtr joint4;

//     transport::NodePtr node;
//     transport::SubscriberPtr sub;
//   };

//   GZ_REGISTER_MODEL_PLUGIN(MultiFloorDoorControl)
// }

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo 
{
  // Fixes the wishbone links as vertical
  class VertWb : public ModelPlugin
  {
      public: VertWb() : ModelPlugin()
        {
        }

      public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
          this->model = _parent;

          this->bleg_r = this->model->GetJoint("j_bleg_r");
          this->bleg_l = this->model->GetJoint("j_bleg_l");
          this->fleg_r = this->model->GetJoint("j_fleg_r");
          this->fleg_l = this->model->GetJoint("j_fleg_l");

          // Initialize the JointController with the model pointer
          this->j_ctrl = new physics::JointController(_parent);

          // Listen to the update event. This event is 
          // broadcast every simulation iteration.
          this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&VertWb::OnUpdate, this, _1));
        }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      double ang_bleg_r = this->bleg_r->GetAngle(2).Radian();
      double ang_bleg_l = this->bleg_l->GetAngle(2).Radian();
      double ang_fleg_r = this->fleg_r->GetAngle(2).Radian();
      double ang_fleg_l = this->fleg_l->GetAngle(2).Radian();
  
      j_ctrl->SetJointPosition("j_bmem_r", -ang_bleg_r);
      j_ctrl->SetJointPosition("j_bmem_l", -ang_bleg_l);
      j_ctrl->SetJointPosition("j_fmem_r", -ang_fleg_r);
      j_ctrl->SetJointPosition("j_fmem_l", -ang_fleg_l);
    }

    private: 

      physics::ModelPtr model;

      physics::JointPtr bleg_r;
      physics::JointPtr bleg_l;
      physics::JointPtr fleg_r;
      physics::JointPtr fleg_l;

      physics::JointController * j_ctrl;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(VertWb);
}

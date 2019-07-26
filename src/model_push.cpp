/*
 * hello_model_plugin.cpp
 *
 *  Created on: Jul 24, 2019
 *      Author: student
 */

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>

namespace gazebo {
class ModelPush : public ModelPlugin {
 private:
  physics::ModelPtr model;
 private:
  event::ConnectionPtr updateConnection;
 public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr) {
	ROS_INFO("push plugin loaded!!!");
	this->model = _parent;
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		std::bind(&ModelPush::OnUpdate, this));
  }

 public:
  void OnUpdate() {
	this->model->SetLinearVel(ignition::math::Vector3d(0.1, 0.0, 0.0));
  }

};

GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

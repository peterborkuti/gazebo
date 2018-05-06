/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <mutex>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Light.hh>
#include <gazebo/rendering/Scene.hh>
#include "LightPublisher.hh"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

namespace gazebo
{
  /// \internal
  /// \class LightPublisher LightPublisher.hh
  /// \brief Private data for the LightPublisher class.
  class LightPublisherPrivate
  {
    /// \brief Visual whose color will be changed.
    public: physics::WorldPtr worldPtr;

    /// \brief Time taken by a full cycle.
    public: common::Time nextTick;

    public: bool lightIsOn;

    /// \brief Connects to rendering update event.
    public: event::ConnectionPtr updateConnection;

    /// \brief Connects to ResetTimer event.
    public: event::ConnectionPtr resetConnection;

    /// \brief Dunno what it is.
    public: std::mutex mutex;
  };
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LightPublisher)

/////////////////////////////////////////////////
LightPublisher::LightPublisher() : dataPtr(new LightPublisherPrivate)
{
}

/////////////////////////////////////////////////
LightPublisher::~LightPublisher()
{
}

/////////////////////////////////////////////////
void LightPublisher::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!_model || !_sdf)
  {
    gzerr << "No model or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }

  this->dataPtr->worldPtr = _model->GetWorld();

  //this->dataPtr->lightPtr = _visual->GetScene()->GetLight("user_spot_light_0");

  if (!this->dataPtr->worldPtr) {
      gzerr << "No World found. Plugin won't load." <<
          std::endl;
      return;
  }

  // Namespace
  if (!_sdf->HasElement("namespace"))
  {
    gzerr << "Missing required parameter <namespace>" << std::endl;
    return;
  }

  this->ns = _sdf->Get<std::string>("namespace");

  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  this->lightSwitcherPub = this->gzNode->Advertise<msgs::Int>(
      "/" + this->ns + "/lightswitcher");

  this->dataPtr->lightIsOn = true;

  this->dataPtr->nextTick = common::Time::Zero;

  // Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LightPublisher::Update, this));

  this->dataPtr->resetConnection = event::Events::ConnectTimeReset(
      std::bind(&LightPublisher::ResetTime, this));

  gzmsg << "LightPublisher started" << std::endl;
}

/////////////////////////////////////////////////
void LightPublisher::ResetTime()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  gzmsg << "LightPublisher::ResetTime" << std::endl;

  this->dataPtr->nextTick = common::Time::Zero;
}

/////////////////////////////////////////////////
void LightPublisher::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  //gzmsg << "LightPublisher::Update" << std::endl;

  if (!this->dataPtr->worldPtr)
  {
    gzerr << "The World is null." << std::endl;
    return;
  }

  common::Time currentTime = this->dataPtr->worldPtr->SimTime();

  //gzmsg << "currentTime:" << currentTime << std::endl;

  if (this->dataPtr->nextTick <= currentTime)
  {

     // secs.nanosecs
     double interval = ignition::math::Rand::DblUniform(0, 5);
     this->dataPtr->nextTick = currentTime + common::Time(interval);

     this->dataPtr->lightIsOn = !this->dataPtr->lightIsOn;

     this->SendMessage((this->dataPtr->lightIsOn) ? 0 : 1000);
  }
}

/////////////////////////////////////////////////
void LightPublisher::SendMessage(int range)
{
    //gzmsg << "LightPublisher::SendMessage:" << range << std::endl;
    gazebo::msgs::Int msg;
    msg.set_data(range);
    this->lightSwitcherPub->Publish(msg);

}

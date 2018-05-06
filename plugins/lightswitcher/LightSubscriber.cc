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
#include "LightSubscriber.hh"
#include <gazebo/physics/Model.hh>

namespace gazebo
{
  /// \internal
  /// \class LightSubscriber LightSubscriber.hh
  /// \brief Private data for the LightSubscriber class.
  class LightSubscriberPrivate
  {
    /// \brief Visual whose color will be changed.
    public: rendering::LightPtr lightPtr;

    /// \brief Time taken by a full cycle.
    public: common::Time nextTick;

    public: bool lightIsOn;

    /// \brief Connects to rendering update event.
    public: event::ConnectionPtr updateConnection;


    /// \brief Dunno what it is.
    public: std::mutex mutex;
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(LightSubscriber)

/////////////////////////////////////////////////
LightSubscriber::LightSubscriber() : dataPtr(new LightSubscriberPrivate)
{
}

/////////////////////////////////////////////////
LightSubscriber::~LightSubscriber()
{
}

/////////////////////////////////////////////////
void LightSubscriber::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }

  this->dataPtr->lightPtr = _visual->GetScene()->GetLight("user_spot_light_0");

  if (!this->dataPtr->lightPtr) {
      gzerr << "No Light found. Plugin won't load." <<
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

  this->lightSwitcherSub = this->gzNode->Subscribe("/" + this->ns + "/lightswitcher",
      &LightSubscriber::Update, this);

    gzmsg << "LightSubscriber started" << std::endl;
}

/////////////////////////////////////////////////
void LightSubscriber::Update(ConstIntPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  int range = _msg->data();

  //gzmsg << "Got message:" << range << std::endl;

  if (!this->dataPtr->lightPtr)
  {
    gzerr << "The Light is null." << std::endl;
    return;
  }

  this->dataPtr->lightPtr->SetRange(range);
}


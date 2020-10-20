// Copyright (c) 2020 The Plankton Authors.
// All rights reserved.
//
// This source code is derived from UUV Simulator
// (https://github.com/uuvsimulator/uuv_simulator)
// Copyright (c) 2016-2019 The UUV Simulator Authors
// licensed under the Apache 2 license
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file GaussMarkovProcess.cpp

#include <uuv_world_plugins/GaussMarkovProcess.h>

namespace gazebo
{
//=============================================================================
///////////////////////////////////////////////////////////////////////////////
GaussMarkovProcess::GaussMarkovProcess()
{
  this->mean = 0;
  this->min = -1;
  this->max = 1;
  this->mu = 0;
  this->noiseAmp = 0;
  this->Reset();
  //TODO Use modern rand
  std::srand(std::time(NULL));
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void GaussMarkovProcess::Reset()
{
  this->var = this->mean;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
bool GaussMarkovProcess::SetMean(double _mean)
{
  if (this->min > _mean || this->max < _mean)
    return false;

  this->mean = _mean;
  this->Reset();
  return true;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
bool GaussMarkovProcess::SetModel(double _mean, double _min, double _max,
    double _mu, double _noise)
{
  if (_min > _max)
    return false;
  if (_min > _mean || _max < _mean)
    return false;
  if (_noise < 0)
    return false;
  if (_mu < 0 || _mu > 1)
    return false;
  this->mean = _mean;
  this->min = _min;
  this->max = _max;
  this->mu = _mu;
  this->noiseAmp = _noise;

  this->Reset();
  return true;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
double GaussMarkovProcess::Update(double _time)
{
  double step = _time - this->lastUpdate;
  double random =  static_cast<double>(static_cast<double>(rand()) / RAND_MAX)
    - 0.5;
  this->var = (1 - step * this->mu) * this->var + this->noiseAmp * random;
  if (this->var >= this->max)
    this->var = this->max;
  if (this->var <= this->min)
    this->var = this->min;
  this->lastUpdate = _time;
  return this->var;
}

//=============================================================================
///////////////////////////////////////////////////////////////////////////////
void GaussMarkovProcess::Print()
{
  gzmsg << "\tMean = " << this->mean << std::endl
    << "\tMin. Limit = " << this->min << std::endl
    << "\tMax. Limit = " << this->max << std::endl
    << "\tMu = " << this->mu << std::endl
    << "\tNoise Amp. = " << this->noiseAmp << std::endl;
}
}

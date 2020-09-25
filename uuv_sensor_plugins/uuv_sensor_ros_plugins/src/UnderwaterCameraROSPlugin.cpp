// Copyright (c) 2020 The Plankton Authors.
// All rights reserved.
//
// This source code is derived from UUV Simulator
// (https://github.com/uuvsimulator/uuv_simulator)
// Copyright (c) 2016-2019 The UUV Simulator Authors
// licensed under the Apache 2 license
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// #include <uuv_sensor_ros_plugins/UnderwaterCameraROSPlugin.h>

// #include <gazebo_plugins/gazebo_ros_camera.hpp>

// namespace gazebo
// {
// /////////////////////////////////////////////////
// UnderwaterCameraROSPlugin::UnderwaterCameraROSPlugin()
//   : lastImage(NULL)
// { }

// /////////////////////////////////////////////////
// UnderwaterCameraROSPlugin::~UnderwaterCameraROSPlugin()
// {
//   if (this->lastImage)
//     delete[] lastImage;

//   if (this->depth2rangeLUT)
//     delete[] depth2rangeLUT;
// }

// /////////////////////////////////////////////////
// void UnderwaterCameraROSPlugin::Load(sensors::SensorPtr _sensor,
//   sdf::ElementPtr _sdf)
// {
//   try
//   {
//     //DepthCameraPlugin::Load(_sensor, _sdf);
    
//     // Copying from DepthCameraPlugin into GazeboRosCameraUtils
//     //this->parentSensor_ = this->parentSensor;
//     //this->width_ = this->width;
//     //this->height_ = this->height;
//     //this->depth_ = this->depth;
//     //this->format_ = this->format;
//     //this->camera_ = this->depthCamera;

//     gazebo_plugins::GazeboRosCamera::Load(_sensor, _sdf);
//   }
//   catch(gazebo::common::Exception &_e)
//   {
//     gzerr << "Error loading UnderwaterCameraROSPlugin" << std::endl;
//     return;
//   }

//   // if (!ros::isInitialized())
//   // {
//   //   gzerr << "Not loading UnderwaterCameraROSPlugin since ROS has not "
//   //     << " been properly initialized." << std::endl;
//   //   return;
//   // }

//   lastImage = new unsigned char[GetWidth() * GetHeight() * this->depth];

//   // Only need to load settings specific to this sensor.
//   GetSDFParam<float>(_sdf, "attenuationR", this->attenuation[0], 1.f / 30.f);
//   GetSDFParam<float>(_sdf, "attenuationG", this->attenuation[1], 1.f / 30.f);
//   GetSDFParam<float>(_sdf, "attenuationB", this->attenuation[2], 1.f / 30.f);

//   this->background[0] = (unsigned char)0;
//   this->background[1] = (unsigned char)0;
//   this->background[2] = (unsigned char)0;

//   if (_sdf->HasElement("backgroundR"))
//     this->background[0] = (unsigned char)_sdf->GetElement(
//       "backgroundR")->Get<int>();
//   if (_sdf->HasElement("backgroundG"))
//     this->background[1] = (unsigned char)_sdf->GetElement(
//       "backgroundG")->Get<int>();
//   if (_sdf->HasElement("backgroundB"))
//     this->background[2] = (unsigned char)_sdf->GetElement(
//       "backgroundB")->Get<int>();
//   // Compute camera intrinsics fx, fy from FOVs:
// #if GAZEBO_MAJOR_VERSION >= 7
//   ignition::math::Angle hfov = ignition::math::Angle(depthCamera->HFOV().Radian());
//   ignition::math::Angle vfov = ignition::math::Angle(depthCamera->VFOV().Radian());
// #else
//   ignition::math::Angle hfov = this->depthCamera->GetHFOV();
//   ignition::math::Angle vfov = this->depthCamera->GetVFOV();
// #endif

//   double fx = (0.5* GetWidth()) / tan(0.5 * hfov.Radian());
//   double fy = (0.5* GetHeight()) / tan(0.5 * vfov.Radian());

//   // Assume the camera's principal point to be at the sensor's center:
//   double cx = 0.5 * GetWidth();
//   double cy = 0.5 * GetHeight();

//   // Create and fill depth2range LUT
//   this->depth2rangeLUT = new float[this->width * this->height];
//   float * lutPtr = this->depth2rangeLUT;
//   for (int v = 0; v < GetHeight(); v++)
//   {
//       double y_z = (v - cy)/fy;
//       for (int u = 0; u < GetWidth(); u++)
//       {
//           double x_z = (u - cx)/fx;
//           // Precompute the per-pixel factor in the following formula:
//           // range = || (x, y, z) ||_2
//           // range = || z * (x/z, y/z, 1.0) ||_2
//           // range = z * || (x/z, y/z, 1.0) ||_2
//           *(lutPtr++) = sqrt(1.0 + x_z*x_z + y_z*y_z);
//       }
//   }
// }

// /////////////////////////////////////////////////
// void UnderwaterCameraROSPlugin::OnNewDepthFrame(const float *_image,
//   unsigned int _width, unsigned int _height, unsigned int _depth,
//   const std::string &_format)
// {
//     // TODO: Can we assume this pointer to always remain valid?
//     this->lastDepth = _image;
// }

// /////////////////////////////////////////////////
// void UnderwaterCameraROSPlugin::OnNewRGBPointCloud(const float * _pcd,
//   unsigned int _width, unsigned int _height, unsigned int _depth,
//   const std::string &_format)
// { }

// /////////////////////////////////////////////////
// void UnderwaterCameraROSPlugin::OnNewImageFrame(const unsigned char *_image,
//   unsigned int _width, unsigned int _height, unsigned int _depth,
//   const std::string& _format)
// {
//   // Only create cv::Mat wrappers around existing memory
//   // (neither allocates nor copies any images).
//   const cv::Mat input(_height, _width, CV_8UC3,
//     const_cast<unsigned char*>(_image));
//   const cv::Mat depth(_height, _width, CV_32FC1,
//     const_cast<float*>(lastDepth));

//   cv::Mat output(_height, _width, CV_8UC3, lastImage);

//   this->SimulateUnderwater(input, depth, output, _width, _height);

//   if (!this->initialized_ || _height <= 0 || _width <= 0)
//     return;

// // #if GAZEBO_MAJOR_VERSION >= 7
// //   this->sensor_update_time_ = this->parentSensor->LastUpdateTime();
// // #else
// //   this->sensor_update_time_ = this->parentSensor->GetLastUpdateTime();
// // #endif

//   if (!this->parentSensor->IsActive())
//   {
//     //if ((*this->image_connect_count_) > 0)
//       // Do this first so there's chance for sensor to run 1 frame after
//       // activate
//       this->parentSensor->SetActive(true);
//   }
//   // else //I guess publish is automatically handled now
//   // {
//   //   if ((*this->image_connect_count_) > 0)
//   //     this->PutCameraData(this->lastImage);
//   //   this->PublishCameraInfo();
//   // }

//   gazebo_plugins::GazeboRosCamera::OnNewImageFrame(_image, _width, _height, _depth, _format);
// }

// /////////////////////////////////////////////////
// void UnderwaterCameraROSPlugin::SimulateUnderwater(const cv::Mat& _inputImage,
//   const cv::Mat& _inputDepth, cv::Mat& _outputImage, unsigned int width, unsigned int height)
// {
//   const float * lutPtr = this->depth2rangeLUT;
//   for (unsigned int row = 0; row < height; row++)
//   {
//     const cv::Vec3b* inrow = _inputImage.ptr<cv::Vec3b>(row);
//     const float* depthrow = _inputDepth.ptr<float>(row);
//     cv::Vec3b* outrow = _outputImage.ptr<cv::Vec3b>(row);

//     for (int col = 0; col < width; col++)
//     {
//       // Convert depth to range using the depth2range LUT
//       float r = *(lutPtr++)*depthrow[col];
//       const cv::Vec3b& in = inrow[col];
//       cv::Vec3b& out = outrow[col];

//       if (r < 1e-3)
//         r = 1e10;

//       for (int c = 0; c < 3; c++)
//       {
//         // Simplifying assumption: intensity ~ irradiance.
//         // This is not really the case but a good enough approximation
//         // for now (it would be better to use a proper Radiometric
//         // Response Function).
//         float e = std::exp(-r*attenuation[c]);
//         out[c] = e*in[c] + (1.0f-e)*background[c];
//       }
//     }
//   }
// }

// /////////////////////////////////////////////////
// GZ_REGISTER_SENSOR_PLUGIN(UnderwaterCameraROSPlugin)
// }

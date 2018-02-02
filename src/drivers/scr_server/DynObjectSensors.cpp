/***************************************************************************

    file                 : DynObjectSensors.cpp
    copyright            : (C) 2018 Florian Mirus
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "DynObjectSensors.h"

DynObjectSensor::DynObjectSensor(tTrack* track, tCarElt* car, tSituation *situation, int range)
{
  ego_car_ = car;
  track_ = track;
  situation_ = situation;
  sensor_range_ = range;
}

DynObjectSensor::~DynObjectSensor()
{
  delete [] ego_car_;
  delete [] track_;
  delete [] situation_;
}

void DynObjectSensor::updateSensors(tSituation *situation)
{
  dyn_obj_vec_.clear();
  for (int i = 0; i < situation->_ncars && situation->_ncars != 1; i++)
	{
		// Compute the distance between my car and the opponent in the (x,y) space
    DynObject current_object;
    current_object.x = situation->cars[i]->pub.DynGC.pos.x - ego_car_->pub.DynGC.pos.x;
    current_object.y = situation->cars[i]->pub.DynGC.pos.y - ego_car_->pub.DynGC.pos.y;
    current_object.z = situation->cars[i]->pub.DynGC.pos.z;

    current_object.roll = constrainAngle(situation->cars[i]->pub.DynGC.pos.ax - ego_car_->pub.DynGC.pos.ax);
    current_object.pitch = constrainAngle(situation->cars[i]->pub.DynGC.pos.ay - ego_car_->pub.DynGC.pos.ay);
    current_object.yaw = constrainAngle(situation->cars[i]->pub.DynGC.pos.az - ego_car_->pub.DynGC.pos.az);

    current_object.x_vel = situation->cars[i]->pub.DynGC.vel.x;
    current_object.y_vel = situation->cars[i]->pub.DynGC.vel.y;
    current_object.z_vel = situation->cars[i]->pub.DynGC.vel.z;

    current_object.roll_vel = situation->cars[i]->pub.DynGC.vel.ax;
    current_object.pitch_vel = situation->cars[i]->pub.DynGC.vel.ay;
    current_object.yaw_vel = situation->cars[i]->pub.DynGC.vel.az;

    // calculate euclidean distance between the ego-vehicle and the current dynamic object
    double dist = sqrt(pow(current_object.x,2) + pow(current_object.y,2));
    // only add this object if it is within sensor range
    if(dist < sensor_range_)
    {
      dyn_obj_vec_.push_back(current_object);
    }
	}
}

float* DynObjectSensor::objectVec2FloatArray()
{
  int nb_objects = dyn_obj_vec_.size();
  float result[12 * nb_objects];

  if (!dyn_obj_vec_.empty())
  {
    for (size_t i = 0; i < dyn_obj_vec_.size(); i++)
    {
      result[12*i+0] = dyn_obj_vec_[i].x;
      result[12*i+1] = dyn_obj_vec_[i].y;
      result[12*i+2] = dyn_obj_vec_[i].z;
      result[12*i+3] = dyn_obj_vec_[i].roll;
      result[12*i+4] = dyn_obj_vec_[i].pitch;
      result[12*i+5] = dyn_obj_vec_[i].yaw;

      result[12*i+6] = dyn_obj_vec_[i].x_vel;
      result[12*i+7] = dyn_obj_vec_[i].y_vel;
      result[12*i+8] = dyn_obj_vec_[i].z_vel;
      result[12*i+9] = dyn_obj_vec_[i].roll_vel;
      result[12*i+10] = dyn_obj_vec_[i].pitch_vel;
      result[12*i+11] = dyn_obj_vec_[i].yaw_vel;
    }
  }
  else
  {
    memset(result, 0.0, 12*nb_objects);
  }
  return result;
}

int DynObjectSensor::getFloatLength()
{
  return (12 * dyn_obj_vec_.size());
}

void DynObjectSensor::addNoise(double noise)
{
  if (!dyn_obj_vec_.empty())
  {
    for (size_t i = 0; i < dyn_obj_vec_.size(); i++)
    {
      dyn_obj_vec_[i].x *= noise;
      dyn_obj_vec_[i].y *= noise;
      dyn_obj_vec_[i].z *= noise;
      dyn_obj_vec_[i].roll *= noise;
      dyn_obj_vec_[i].pitch *= noise;
      dyn_obj_vec_[i].yaw *= noise;

      dyn_obj_vec_[i].x_vel *= noise;
      dyn_obj_vec_[i].y_vel *= noise;
      dyn_obj_vec_[i].z_vel *= noise;
      dyn_obj_vec_[i].roll_vel *= noise;
      dyn_obj_vec_[i].pitch_vel *= noise;
      dyn_obj_vec_[i].yaw_vel *= noise;
    }
  }
}

double DynObjectSensor::constrainAngle(double x){
  x = fmod(x + PI,2*PI);
  if (x < 0){
    x += 2*PI;
  }
  return x - PI;
}

std::vector<DynObject> DynObjectSensor::getObjectList()
{
  return dyn_obj_vec_;
}

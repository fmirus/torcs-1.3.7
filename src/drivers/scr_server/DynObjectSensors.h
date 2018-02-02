/***************************************************************************

    file                 : DynObjectSensors.h
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

#ifndef _DYN_OBJ_SENS_H_
#define _DYN_OBJ_SENS_H_

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <car.h>
#include <robottools.h>
#include <raceman.h>
#include <vector>

struct DynObject{
public:
  double x;
  double y;
  double z;

  double roll;
  double pitch;
  double yaw;

  double x_vel;
  double y_vel;
  double z_vel;

  double roll_vel;
  double pitch_vel;
  double yaw_vel;

  DynObject()
  {
    x = 0;
    y = 0;
    z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;

    x_vel = 0;
    y_vel = 0;
    z_vel = 0;
    roll_vel = 0;
    pitch_vel = 0;
    yaw_vel = 0;
  }
};

class DynObjectSensor{
public:
  DynObjectSensor(tTrack* track, tCarElt* car, tSituation *situation, int range);
  ~DynObjectSensor();

  void updateSensors(tSituation *situation);

  float* objectVec2FloatArray();

  void addNoise(double noise);
  double constrainAngle(double x);
  int getFloatLength();

  std::vector<DynObject> getObjectList();

private:
  tCarElt* ego_car_;
  tTrack* track_;
  tSituation* situation_;
  int sensor_range_;
  std::vector<DynObject> dyn_obj_vec_;
};

#endif

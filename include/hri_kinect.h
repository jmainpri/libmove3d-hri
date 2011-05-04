/*
 *  hri_kinect.h
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 06/01/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef _HRI_KINECT_H
#define _HRI_KINECT_H

#include "hri_agent.h"

struct kinectJoint {
  p3d_vector3 pos;
  double      confidence;
}

// Only HEAD, NECK, TORSO, 
// SHOULDERS, ELBOW, 
// HAND, HIP, KNEE and FOOT 
struct kinectData 
{
  kinectJoint HEAD, NECK, TORSO;
  kinectJoint SHOULDER_RIGHT;
  kinectJoint SHOULDER_LEFT;
  kinectJoint ELBOW_RIGHT;
  kinectJoint ELBOW_LEFT;
  kinectJoint HIP_LEFT;
  kinectJoint HIP_RIGHT;
  kinectJoint HAND_RIGHT;
  kinectJoint HAND_LEFT;
  kinectJoint KNEE_RIGHT;
  kinectJoint KNEE_LEFT;
  kinectJoint FOOT_RIGHT;
  kinectJoint FOOT_LEFT;
};

struct kinectAgent 
{
  int kinectId;
  kinectData data;
  HRI_AGENT* agent;
};

#define KINECT_MAX_NUM_HUMANS 16

struct kinectAgents 
{
  int num;
  kinectAgent humans[KINECT_MAX_NUM_HUMANS];
};

configPt hri_get_configuration_from_kinect_data( p3d_rob* robot, kinectData& data );
void hri_store_kinect_model( kinectData& data );
void hri_draw_kinect_points();

#endif

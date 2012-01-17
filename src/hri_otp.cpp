//
//  hri_otp.cpp
//  libhri
//
//  Created by Jim Mainprice on 17/01/12.
//  Copyright (c) 2012 LAAS/CNRS. All rights reserved.
//

#include "hri.h"

#include <iostream>

//! get a OTP for object transfer point
bool hri_get_best_otp_position( p3d_rob* rob, p3d_rob* human, p3d_vector3 otp )
{
  p3d_vector3 tmp1,tmp2,robot_center,wrist_center,human_center;  
  
  robot_center[0] = rob->joints[1]->abs_pos[0][3];
  robot_center[1] = rob->joints[1]->abs_pos[1][3];
  robot_center[2] = 0;
  
  human_center[0] = human->joints[1]->abs_pos[0][3];
  human_center[1] = human->joints[1]->abs_pos[1][3];
  human_center[2] = 0;
  
  if( p3d_vectDistance( robot_center, human_center ) > 1.2 )
  {
    robot_center[2] = 1.0;
    human_center[2] = 1.0;
    
    p3d_vectSub( human_center, robot_center, tmp1 );
    p3d_vectNormalize( tmp1 , tmp2 );
    
    // A point at 80 cm from the robot
    p3d_vectScale( tmp2 , tmp1, 0.8 );
    p3d_vectAdd( tmp1, robot_center, otp );
  }
  else
  {
    p3d_jnt* wrist = p3d_get_robot_jnt_by_name( rob, "rWristX" );
    
    if( wrist == NULL )
    {
      printf("Human right wrist not found\n");
      return false;
    }
    
    wrist_center[0] = wrist->abs_pos[0][3];
    wrist_center[1] = wrist->abs_pos[1][3];
    wrist_center[2] = wrist->abs_pos[2][3];
    
    // The point above the robot center at the hieght
    // of the human wrist
    robot_center[2] =  wrist_center[2];
    
    p3d_vectSub( robot_center, wrist_center, tmp1 );
    p3d_vectNormalize( tmp1 , tmp2 );
    
    // We define the point at 10cm along tmp2
    p3d_vectScale( tmp2 , tmp1, 0.10 );
    p3d_vectAdd( tmp1, wrist_center, otp );
  }
}

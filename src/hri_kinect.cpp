
/*
 *  hri_kinect.cpp
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 06/01/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#include "hri.h"
#include "P3d-pkg.h"

using namespace std;

//! @ingroup KINECT 
//! human kinect information
bool        m_debug_kinect = false;
kinectState m_state = KINECT_DISCONNECTED;
bool        m_data_exists = false;
kinectData  m_humKin;
p3d_rob*    m_human;
p3d_matrix4 m_absPos;

//! @ingroup KINECT 
//! set the flag for debug
void hri_set_debug_kinect(bool is_debug)
{
  m_debug_kinect = is_debug;
}

//! @ingroup KINECT 
//! Sores the kinect state
void hri_set_kinect_state(const kinectState& state)
{
  m_state = state;
}

//! @ingroup KINECT 
//! set the human position form the kinect information
//! right arm 1,2,3
//! left arm 4,5,6
void hri_set_human_config_from_kinect( HRI_AGENTS * agents , configPt q , kinectData& humKin )
{
  p3d_rob * robot = NULL;
//  p3d_jnt* joint = NULL;
//  int index_dof;
  
  if (!q) {
    printf("Error : hri_set_human_config_from_kinect\n");
  }
  
  // gets the first human robot
  for(int i=0; i<agents->all_agents_no; i++) 
  {
    if( agents->all_agents[i]->is_human )
    {
      robot = agents->all_agents[i]->robotPt;
      break;
    }
  }
  
  //return FALSE;
}

//! @ingroup KINECT 
//! Computes a condfiguration
//! from a set of points 
configPt hri_get_configuration_from_kinect_data( p3d_rob* robot, kinectData& data )
{
  m_human = robot;

  // Set pelvis
  p3d_jnt* joint = p3d_get_robot_jnt_by_name(robot, (char*) "Pelvis");
  int index_dof = joint->index_dof;
  
  configPt q = p3d_get_robot_config(robot);

  p3d_vector3 pos;

  if( data.TORSO.confidence > 0 && data.HIP_LEFT.confidence > 0 && data.HIP_RIGHT.confidence > 0 )
    {
      q[index_dof+0] = data.TORSO.pos[0];
      q[index_dof+1] = data.TORSO.pos[1];
      q[index_dof+2] = data.TORSO.pos[2] - 0.20 ; // Hack 1 meter
  
      // calcul de l'orientation du pelvis
      p3d_vector3 sum,midP;
      p3d_vectAdd( data.HIP_LEFT.pos, data.HIP_RIGHT.pos , sum );
      p3d_vectScale(  sum , midP , 0.5 );
  
      q[index_dof+5] = atan2( data.HIP_LEFT.pos[1]-midP[1] , data.HIP_LEFT.pos[0]-midP[0]  );
      q[index_dof+5] += -M_PI/2; // Hack +  Pi / 2  
    }

  //--------------------------------------------------------------
  p3d_matrix4 Tinv,Tpelv;
  p3d_vector3 shoulder;
  p3d_vector3 Xaxis = { 1 , 0 , 0 };
  p3d_matrix4 TrotX,TrotXtmp;
  p3d_vector3 Yaxis = { 0 , 1 , 0 };
  p3d_matrix4 TrotY,TrotYtmp;

  if( data.NECK.confidence > 0 && data.SHOULDER_LEFT.confidence > 0 )
    {
      p3d_set_and_update_this_robot_conf( robot, q );  
      joint = p3d_get_robot_jnt_by_name(robot, (char*) "Pelvis");


      p3d_mat4Copy( joint->abs_pos , Tpelv );
      p3d_mat4Copy( joint->abs_pos , m_absPos );
      p3d_matInvertXform( Tpelv , Tinv  );
      p3d_xformPoint( Tinv , data.NECK.pos , pos );
  
      double TorsoX = atan2( -pos[1] , pos[2] );
      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "TorsoX")->index_dof; 
      q[index_dof] = TorsoX;


      p3d_mat4TransRot( TrotX , 0 , 0 , 0 , Xaxis , TorsoX  );
      p3d_matMultXform( Tpelv , TrotX , TrotXtmp );
      p3d_matInvertXform( TrotXtmp , Tinv  );
      p3d_xformPoint( Tinv , data.NECK.pos , pos );

      double TorsoY = atan2( pos[0] , pos[2] );
      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "TorsoY")->index_dof; 
      q[index_dof] = TorsoY;


      p3d_mat4TransRot( TrotY , 0 , 0 , 0 , Yaxis , TorsoY  );
      p3d_matMultXform( TrotXtmp , TrotY , TrotYtmp );
      p3d_matInvertXform( TrotYtmp , Tinv  );
      p3d_xformPoint( Tinv , data.SHOULDER_LEFT.pos , pos );

      double TorsoZ = atan2( -pos[0] , pos[1] );
      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "TorsoZ")->index_dof; 
      q[index_dof] = TorsoZ;
    }

  // -----------------------------------------------------------------
  // Set and update robot to 
  // new position
  // -----------------------------------------------------------------
  p3d_matrix4 Trot;
  p3d_matrix4 Trot2,Trot3;
  p3d_vector3 Zaxis = { 0 , 0 , 1 };
  p3d_matrix4 Trot4;
  p3d_vector3 vect1,vect2,vect3;

  if( data.ELBOW_RIGHT.confidence > 0 && data.HAND_RIGHT.confidence > 0 )
    {
      p3d_set_and_update_this_robot_conf( robot, q );

      joint = p3d_get_robot_jnt_by_name(robot, (char*) "rShoulderX");
      p3d_jnt_get_cur_vect_point( joint , pos );
      p3d_jnt_get_cur_vect_point( joint , shoulder );
 
      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rArmTrans")->index_dof; 
      q[index_dof] = p3d_vectDistance( data.ELBOW_RIGHT.pos , pos ) - 0.2066 ;

 
      joint = p3d_get_robot_jnt_by_name(robot, (char*) "TorsoZ");
      p3d_mat4Copy( joint->abs_pos , Trot );
      Trot[0][3] = pos[0];
      Trot[1][3] = pos[1];
      Trot[2][3] = pos[2];

      //  p3d_mat4Copy( Trot , m_absPos );

      p3d_matInvertXform( Trot , Tinv  );
      p3d_xformPoint( Tinv , data.ELBOW_RIGHT.pos , pos );

      // calcul de la direction pour le bras droit
      p3d_vector3 dir,sub;
      p3d_vectNormalize( pos , dir );

      // JP
      // selon x
      double alpha1r = atan2( -pos[2] , -pos[1] );
 
      p3d_mat4TransRot( Trot2 , 0 , 0 , 0 , Xaxis , alpha1r );
      p3d_matMultXform( Trot , Trot2 , Trot3 );
      p3d_matInvertXform( Trot3 , Tinv  );
      p3d_xformPoint( Tinv , data.ELBOW_RIGHT.pos , pos );

      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rShoulderX")->index_dof;
      q[index_dof] = alpha1r;

      // selon z
      double alpha2r = atan2( pos[0] , -pos[1] );

      //  printf("alpha1 = %f\n", alpha1r );
      //  printf("alpha2 = %f\n", alpha2r );
      //  printf("dir = ( %f , %f , %f )\n", pos[0] , pos[1] , pos[2] );
  

      p3d_mat4TransRot( Trot2 , 0 , 0 , 0 , Zaxis , alpha2r );
      p3d_matMultXform( Trot3 , Trot2 , Trot4 );
      p3d_matInvertXform( Trot4 , Tinv  );
      p3d_xformPoint( Tinv , data.HAND_RIGHT.pos , pos );

      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rShoulderZ")->index_dof;
      q[index_dof] = alpha2r;

      // selon y  
      double alpha3r = atan2( pos[2], pos[0] );  

      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rShoulderY")->index_dof; 
      q[index_dof] = alpha3r; 

  
      p3d_vectSub( shoulder ,  data.ELBOW_RIGHT.pos , vect1 );
      p3d_vectNormalize( vect1 , vect1 );

      p3d_vectSub( data.HAND_RIGHT.pos , data.ELBOW_RIGHT.pos , vect2 );
      p3d_vectNormalize( vect2 , vect2 );

      // Elbow
      double alpha4r = M_PI - acos( p3d_vectDotProd( vect1 , vect2) ) ;

      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rElbowZ")->index_dof; 
      q[index_dof] = alpha4r; 
    }

  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  if( data.ELBOW_LEFT.confidence > 0 && data.HAND_LEFT.confidence > 0 )
    {
      joint = p3d_get_robot_jnt_by_name(robot, (char*) "lShoulderX");
      p3d_jnt_get_cur_vect_point( joint , pos );
      p3d_jnt_get_cur_vect_point( joint , shoulder );

      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "lArmTrans")->index_dof; 
      q[index_dof] = p3d_vectDistance( data.ELBOW_LEFT.pos , pos ) - 0.2066 ;

      joint = p3d_get_robot_jnt_by_name(robot, (char*) "TorsoZ");
      p3d_mat4Copy( joint->abs_pos , Trot );
      Trot[0][3] = pos[0];
      Trot[1][3] = pos[1];
      Trot[2][3] = pos[2];

      //  p3d_mat4Copy( Trot , m_absPos );


      p3d_matInvertXform( Trot , Tinv  );
      p3d_xformPoint( Tinv , data.ELBOW_LEFT.pos , pos );

      // JP
      // selon x
      double alpha1l = atan2( pos[2] , pos[1] );

      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "lShoulderX")->index_dof;
      q[index_dof] = alpha1l;

      p3d_mat4TransRot( Trot2 , 0 , 0 , 0 , Xaxis , alpha1l );
      p3d_matMultXform( Trot , Trot2 , Trot3 );
      p3d_matInvertXform( Trot3 , Tinv  );
      p3d_xformPoint( Tinv , data.ELBOW_LEFT.pos , pos );

      // selon z
      double alpha2l = atan2( -pos[0] , pos[1] );

      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "lShoulderZ")->index_dof;
      q[index_dof] = alpha2l;

      //  printf("alpha1 = %f\n", alpha1l );
      //  printf("alpha2 = %f\n", alpha2l );
      //  printf("dir = ( %f , %f , %f )\n", pos[0] , pos[1] , pos[2] );

      p3d_mat4TransRot( Trot2 , 0 , 0 , 0 , Zaxis , alpha2l );
      p3d_matMultXform( Trot3 , Trot2 , Trot4 );
      p3d_matInvertXform( Trot4 , Tinv  );
      p3d_xformPoint( Tinv , data.HAND_LEFT.pos , pos );

      // selon y  
      double alpha3l = -atan2( -pos[2], pos[0] );  
  
      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "lShoulderY")->index_dof; 
      q[index_dof] = alpha3l; 

      p3d_vectSub( shoulder ,  data.ELBOW_LEFT.pos , vect1 );
      p3d_vectNormalize( vect1 , vect1 );

      p3d_vectSub( data.HAND_LEFT.pos , data.ELBOW_LEFT.pos , vect2 );
      p3d_vectNormalize( vect2 , vect2 );

      double alpha4l = -M_PI + acos( p3d_vectDotProd( vect1 , vect2) ) ;

      index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "lElbowZ")->index_dof; 
      q[index_dof] = alpha4l; 
    }

  return q;
}

//! @ingroup KINECT 
void hri_store_kinect_model( kinectData& data )
{
  m_data_exists = true;
  m_humKin = data;
}

//! @ingroup KINECT 
void hri_draw_kinect_points()
{
  if (!m_data_exists) 
  {
    return;
  }
  
//  g3d_draw_frame( m_absPos , 0.5 );
  
  if(m_debug_kinect)
  {
    cout << "---------------------------------" << endl;
    cout << "HEAD = " <<  m_humKin.HEAD.confidence << endl;
    cout << "NECK = " << m_humKin.NECK.confidence << endl;
    cout << "TORSO = " << m_humKin.TORSO.confidence << endl;
    cout << "SHOULDER_RIGHT = " << m_humKin.SHOULDER_RIGHT.confidence << endl;
    cout << "SHOULDER_LEFT = " << m_humKin.SHOULDER_LEFT.confidence << endl;
    cout << "ELBOW_RIGHT = " << m_humKin.ELBOW_RIGHT.confidence << endl;
    cout << "ELBOW_LEFT = " << m_humKin.ELBOW_LEFT.confidence << endl;
    cout << "HAND_RIGHT = " << m_humKin.HAND_RIGHT.confidence << endl;
    cout << "HAND_LEFT = " << m_humKin.HAND_LEFT.confidence << endl;
    cout << "HIP_RIGHT = " << m_humKin.HIP_RIGHT.confidence << endl;
    cout << "HIP_LEFT = " << m_humKin.HIP_LEFT.confidence << endl;
                 
  double r = 0.03;
  g3d_drawSphere(m_humKin.HEAD.pos[0],m_humKin.HEAD.pos[1],m_humKin.HEAD.pos[2],r);
  g3d_drawSphere(m_humKin.NECK.pos[0],m_humKin.NECK.pos[1],m_humKin.NECK.pos[2],r);
  
  g3d_drawSphere(m_humKin.TORSO.pos[0],
                 m_humKin.TORSO.pos[1],
                 m_humKin.TORSO.pos[2],r);
  
  g3d_drawSphere(m_humKin.SHOULDER_RIGHT.pos[0],
                 m_humKin.SHOULDER_RIGHT.pos[1],
                 m_humKin.SHOULDER_RIGHT.pos[2],r);
  
  g3d_drawSphere(m_humKin.SHOULDER_LEFT.pos[0],
                 m_humKin.SHOULDER_LEFT.pos[1],
                 m_humKin.SHOULDER_LEFT.pos[2],r);

  g3d_drawSphere(m_humKin.ELBOW_RIGHT.pos[0],
                 m_humKin.ELBOW_RIGHT.pos[1],
                 m_humKin.ELBOW_RIGHT.pos[2],r);
  
  g3d_drawSphere(m_humKin.ELBOW_LEFT.pos[0],
                 m_humKin.ELBOW_LEFT.pos[1],
                 m_humKin.ELBOW_LEFT.pos[2],r);

  g3d_drawSphere(m_humKin.HAND_RIGHT.pos[0],
                 m_humKin.HAND_RIGHT.pos[1],
                 m_humKin.HAND_RIGHT.pos[2],r);
  
  g3d_drawSphere(m_humKin.HAND_LEFT.pos[0],
                 m_humKin.HAND_LEFT.pos[1],
                 m_humKin.HAND_LEFT.pos[2],r);

  //  r = 0.1;
  //  glColor3f(0,1,0);
  
  g3d_drawSphere(m_humKin.HIP_RIGHT.pos[0],
                 m_humKin.HIP_RIGHT.pos[1],
                 m_humKin.HIP_RIGHT.pos[2],r);
  
  g3d_drawSphere(m_humKin.HIP_LEFT.pos[0],
                 m_humKin.HIP_LEFT.pos[1],
                 m_humKin.HIP_LEFT.pos[2],r);

  }
  
  //g3d_drawSphere(m_humKin.KNEE_RIGHT[0],
  //               m_humKin.KNEE_RIGHT[1],
  //               m_humKin.KNEE_RIGHT[2],r);
  
  //g3d_drawSphere(m_humKin.KNEE_LEFT[0],
  //               m_humKin.KNEE_LEFT[1],
  //               m_humKin.KNEE_LEFT[2],r);
  
  //g3d_drawSphere(m_humKin.FOOT_RIGHT[0],
  //               m_humKin.FOOT_RIGHT[1],
  //               m_humKin.FOOT_RIGHT[2],r);
  
  //g3d_drawSphere(m_humKin.FOOT_LEFT[0],
  //               m_humKin.FOOT_LEFT[1],
  //               m_humKin.FOOT_LEFT[2],r);
}

//! Displays the Kinect State
//! \param offsetX X position of the logo lower-left corner (from the image lower-left corner)
//! \param offsetY Y position of the logo lower-left corner (from the image lower-left corner)
//! \param widthRatio ratio of the logo width with respect of the OpenGL window width (e.g.: use 0.1 if you want the logo width to be 1/10 of the window width) 
//! \return 0 in case of success, 1 otherwise
const unsigned int LOGO_WIDTH = 10;
const unsigned int LOGO_HEIGHT = 10;
void hri_draw_kinect_state(g3d_states &vs, float offsetXRatio, float offsetY, float widthRatio )
{
//  cout <<  "hri_draw_kinect_state" << endl;
  GLfloat color[4];
  
  switch (m_state) 
  {
    case KINECT_DISCONNECTED: 
     return;

    case KINECT_NO_TRACKING:   color[0] = 1.0; color[1]= 0.0; color[2]= 0.0; color[3]= 1.0;
      break;
      
    case KINECT_POSE_SEARCH:   color[0] = 1.0; color[1]= 0.5; color[2]= 0.0; color[3]= 1.0;
      break;
      
    case KINECT_CALIBRATE:     color[0] = 1.0; color[1]= 1.0; color[2]= 0.0; color[3]= 1.0;
      break;
      
    case KINECT_TRACKING:      color[0] = 0.0; color[1]= 1.0; color[2]= 0.0; color[3]= 1.0;
      break;
      
    default:
      cout << "Error setting kinect color state" << endl;
      break;
  }
  
  GLint viewport[4];
  
  glGetIntegerv(GL_VIEWPORT, viewport);
  int width  = viewport[2];
  int height = viewport[3];
  
  float scale= widthRatio*width/( (float) LOGO_WIDTH);
  float offsetX = offsetXRatio*width;
  
#ifdef USE_SHADERS
  if(vs.enableShaders==TRUE)
  { g3d_no_shader(); }
#endif
  
  glPushAttrib(GL_ENABLE_BIT | GL_TRANSFORM_BIT);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, width, 0, height, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity(); 
  glEnable(GL_BLEND);
  glDisable(GL_LIGHTING); 
  glDisable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);	 
  
  glColor4f(color[0], color[1], color[2], color[3]);
  
  glBegin (GL_QUADS);
  glTexCoord2f(1.0f,1.0f);  glVertex2f(offsetX + scale*LOGO_WIDTH, offsetY);
  glTexCoord2f(1.0f,0.0f);  glVertex2f(offsetX + scale*LOGO_WIDTH, offsetY + scale*LOGO_HEIGHT);
  glTexCoord2f(0.0f,0.0f);  glVertex2f(offsetX, offsetY + scale*LOGO_HEIGHT);
  glTexCoord2f(0.0f,1.0f);  glVertex2f(offsetX, offsetY);
  glEnd(); 
  
  glPopMatrix(); 
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glPopAttrib();
  
#ifdef USE_SHADERS
  if(vs.enableShaders==TRUE)
  { g3d_use_shader(); }
#endif
}

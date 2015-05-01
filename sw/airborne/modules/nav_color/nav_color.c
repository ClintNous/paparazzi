/*
 * Copyright (C) Clint Nous
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/NAV_Clint/NAV_Clint.c"
 * @author Clint Nous
 * navigation functions flightplan
 */

#include "modules/nav_color/nav_color.h"
#include "modules/computer_vision/viewvideo.h"
#include "generated/flight_plan.h"
#include "math/pprz_algebra_float.h"
#include "subsystems/datalink/telemetry.h" 

// void NAV_Clint_init {}
#include "nav_color.h"
#include "state.h"
#include "math.h"

static void send_calculated_heading(void) {
  DOWNLINK_SEND_HEADING_CALCULATED(DefaultChannel, DefaultDevice, &heading_new);
 }
 
void nav_color_init(void){
  register_periodic_telemetry(DefaultPeriodic,"HEADING_CALCULATED",send_calculated_heading);
}

void nav_color_period(void){
  //int width = cnt_obst[0];
  //struct EnuCoor_i waypoints[NB_WAYPOINT];
  
  //read in value of waypoints
  struct EnuCoor_f waypoints[NB_WAYPOINT] = WAYPOINTS_ENU;

  //some function to determine what nextwaypoints should be selected, for now 'p1' is chosen or waypoints 3 see flight_plan.h
  //read in position
  struct FloatVect2 current_pos = {stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y};
  struct FloatVect2 target = {waypoints[3].x, waypoints[3].y};
  struct FloatVect2 pos_diff;
  float current_heading = stateGetNedToBodyEulers_f()->psi;
  
  VECT2_DIFF(pos_diff, target, current_pos);
  float heading_goal_f = atan2f(pos_diff.x, pos_diff.y);
  
  //define image size HAS TO BE CHECKED!
 int image_size[2] = {1280/4, 720/4};
 int image_fow[2] = {74, 59};
 float obst_angle[10];
 float obst_width[10];
 
 //define potential field variables
 float r_new;
 float r_dot_new;
 float r_old = stateGetBodyRates_f()->r;

 
 //Tuning variable for algorithm so CHECK!
 float b_damp = 1; 
 float K_goal = 1;
 float K_obst[10] = {1,1,1,1,1,1,1,1,1,1};
 float c1 = 1;
 float c2 = 1;
 float c3 = 1;
 float c5 = 1;
 float potential_obst = 0; 
 
 //calculate position angle and angular widht of obstacles
  for(int i=0;i<10;i++){
   obst_width[i] = cnt_obst[i]*(float)(image_fow[0]/image_size[0]);
   obst_angle[i] = cnt_obst[10+i]*(float)(image_fow[0]/image_size[0])-(float)0.5*image_fow[0]; 
   
      if (obst_width[i] != 0){
	potential_obst = potential_obst + K_obst[i]*obst_angle[i]*exp(-c3*abs(obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
	
      }
  }
  
 //calculate angular accelaration from potential field
  r_dot_new = -b_damp*r_old -K_goal*(current_heading-heading_goal_f)*(exp(-c1*VECT2_NORM2(pos_diff))+c2) + potential_obst;
  
  //Integrate using simple intgration CHECK for time step!
  r_new = r_old + r_dot_new;
  heading_new = current_heading + r_new;
  
}

bool_t test_color_count(void){
  int color_treshold = 6000;
  if(color_count> color_treshold){
    return TRUE;
  }
  else{
    return FALSE;
  }
}

int32_t nav_color_new_heading90(void){
  safe_heading =0.75;  
} 

int32_t nav_color_new_heading180(void){
  safe_heading = 0.75; 
} 


/*bool_t NavSetNewWayPoint(uint8_t curr, uint8_t dist, uint8_t next, uint8_t heading, uint8_t glob, uint8_t look,uint8_t fz1,uint8_t fz2,uint8_t fz3,uint8_t fz4)
    {
    int32_t s_heading, c_heading;
    // distance in cm's
    // random heading (angle) -32,-16,0,16,32 degrees
    // safe_heading = ((rand() % 5) * 16) - 32;
    // safe_heading = 45; //hack for sim testing
    PPRZ_ITRIG_SIN(s_heading, nav_heading+safe_heading);
    PPRZ_ITRIG_COS(c_heading, nav_heading+safe_heading);
    waypoints[heading].x = waypoints[curr].x + INT_MULT_RSHIFT(dist,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
    waypoints[heading].y = waypoints[curr].y + INT_MULT_RSHIFT(dist,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;

    waypoints[next].x=waypoints[heading].x;
    waypoints[next].y=waypoints[heading].y;
   
    
  return FALSE;
}
*/

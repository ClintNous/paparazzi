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

uint8_t point_index = 0;
struct EnuCoor_f waypoints_OA[NB_WAYPOINT] = WAYPOINTS_ENU;


static void send_calculated_heading(void) {
  DOWNLINK_SEND_HEADING_CALCULATED(DefaultChannel, DefaultDevice, &heading_new, &current_heading, &r_dot_new, &potential_obst_write, obst_angle ,obst_width);
 }
 
void nav_color_init(void){
   register_periodic_telemetry(DefaultPeriodic,"HEADING_CALCULATED",send_calculated_heading);
}

void nav_color_period(void){}


bool_t nav_cal_heading(float dist_oa, uint8_t goal, uint8_t follow, uint8_t wp_heading){
  //int width = cnt_obst[0];
  //struct EnuCoor_i waypoints[NB_WAYPOINT];
  
  //read in value of waypoints
  struct EnuCoor_i wp_tmp_int_oa;
  struct EnuCoor_i wp_tmp_int_oa2;

  //read in position
  struct FloatVect2 current_pos = {stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y};
  struct FloatVect2 target = {waypoints_OA[goal].x, waypoints_OA[goal].y};
  struct FloatVect2 pos_diff;
  current_heading = stateGetNedToBodyEulers_f()->psi;
  
  VECT2_DIFF(pos_diff, target, current_pos);
  float heading_goal_f = atan2f(pos_diff.x, pos_diff.y);
  
  //define image size HAS TO BE CHECKED!
 int image_size[2] = {1280/4, 720/4};
 float image_fow[2] = {1.2915, 2.0297};
 float angle_change;
 
 //define potential field variables
 //float r_new;
 float r_old = stateGetBodyRates_f()->r;

 
 //Tuning variable for algorithm
 float b_damp = 5.5; 
 float K_goal = 3;
 float K_obst = 30.0;
 float c1 = 0.4;
 float c2 = 0.4;
 float c3 = 4.0;
 float c5 = 1.5;
 float potential_obst = 0; 

 
 //define delta t for integration! check response under 0.1 and 1
 float dt = 0.5;
 
 //calculate position angle and angular widht of obstacles
  for(int i=0;i<5;i++){
   obst_width[i] = cnt_obst[i]*(image_fow[0]/(float)image_size[0]);
   obst_angle[i] = ((float)cnt_obst[10+i]+0.5*(float)cnt_obst[i])*(image_fow[0]/(float)image_size[0])-(0.5*image_fow[0]); 
   
      if (cnt_obst[i]>10 && obst_angle[i] != -(0.5*image_fow[0])){
	potential_obst = potential_obst + K_obst*(current_heading-obst_angle[i])*exp(-c3*abs(current_heading - obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
	potential_obst_write = potential_obst;
      }
  }
  
 //calculate angular accelaration from potential field
  r_dot_new = -b_damp*r_old -K_goal*(current_heading-heading_goal_f)*(exp(-c1*VECT2_NORM2(pos_diff))+c2) + potential_obst;
  
  //Integrate using simple intgration CHECK for time step!
  heading_new = current_heading + 0.5*r_dot_new*dt*dt;

  waypoints_OA[follow].x = stateGetPositionEnu_f()->x + sinf(heading_new)*dist_oa;
  waypoints_OA[follow].y = stateGetPositionEnu_f()->y + cosf(heading_new)*dist_oa;
  
  waypoints_OA[wp_heading].x = stateGetPositionEnu_f()->x + sinf(heading_new)*2;
  waypoints_OA[wp_heading].y = stateGetPositionEnu_f()->y + cosf(heading_new)*2;
  
  wp_tmp_int_oa.x = POS_BFP_OF_REAL(waypoints_OA[follow].x);
  wp_tmp_int_oa.y = POS_BFP_OF_REAL(waypoints_OA[follow].y);
  wp_tmp_int_oa.z = POS_BFP_OF_REAL(waypoints_OA[follow].z);
  
  wp_tmp_int_oa2.x = POS_BFP_OF_REAL(waypoints_OA[wp_heading].x);
  wp_tmp_int_oa2.y = POS_BFP_OF_REAL(waypoints_OA[wp_heading].y);
  wp_tmp_int_oa2.z = POS_BFP_OF_REAL(waypoints_OA[wp_heading].z);
   
  
 nav_move_waypoint(follow, &wp_tmp_int_oa);
 nav_move_waypoint(wp_heading, &wp_tmp_int_oa2);
  
 return FALSE;
  
}



bool_t NavCalNewWaypointCircle(uint8_t N_points, float radius, uint8_t target)
    {
       
     float angle = 2*M_PI/N_points*point_index;
     struct EnuCoor_f wp_tmp_float_oa[NB_WAYPOINT] = WAYPOINTS_ENU;
     struct EnuCoor_i wp_tmp_int_oa;
    
     wp_tmp_float_oa[target].x = wp_tmp_float_oa[target].x + sinf(angle)*radius;
     wp_tmp_float_oa[target].y = wp_tmp_float_oa[target].y + cosf(angle)*radius;
     
     wp_tmp_int_oa.x = POS_BFP_OF_REAL(wp_tmp_float_oa[target].x);
     wp_tmp_int_oa.y = POS_BFP_OF_REAL(wp_tmp_float_oa[target].y);
     wp_tmp_int_oa.z = POS_BFP_OF_REAL(wp_tmp_float_oa[target].z);
     

     
     nav_move_waypoint(target, &wp_tmp_int_oa);
     
     
     //PPRZ_ITRIG_SIN(s_heading, angle*point_index);
     //PPRZ_ITRIG_COS(c_heading, angle*point_index);
     
      //int16_t angle;
     //angle = INT32_RAD_OF_DEG((int)(angle_ref) << (INT32_ANGLE_FRAC));
     
     //new_pos_wp_circle->x = stateGetPositionEnu_f()->x + sin(angle*point_index)*radius;
     //new_pos_wp_circle->y = stateGetPositionEnu_f()->y + cos(angle*point_index)*radius;
     //new_pos_wp_circle->z = stateGetPositionEnu_f()->z;
     
     //new_pos_wp_circle->x = stateGetPositionEnu_i()->x + INT_MULT_RSHIFT(radius,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC);
     //new_pos_wp_circle->y = stateGetPositionEnu_i()->y + INT_MULT_RSHIFT(radius,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC);
     //new_pos_wp_circle->z = stateGetPositionEnu_i()->z;
     
     //nav_move_waypoint(target, new_pos_wp_circle);
     
     point_index++;     
     
     return FALSE;
}

// void nav_move_waypoint(uint8_t wp_id, struct EnuCoor_i *new_pos)
// {
//   if (wp_id < nb_waypoint) {
//     VECT3_COPY(waypoints[wp_id], (*new_pos));
//     DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, &wp_id, &(new_pos->x),
//                                &(new_pos->y), &(new_pos->z));
//   }
// }


bool_t NavSetNewWayPoint_CN(float dist, uint8_t target)
    {
    int32_t s_heading, c_heading;

    PPRZ_ITRIG_SIN(s_heading, stateGetNedToBodyEulers_i()->psi+heading_new);
    PPRZ_ITRIG_COS(c_heading, stateGetNedToBodyEulers_i()->psi+heading_new);
    
    
    waypoints[target].x = stateGetPositionEnu_f()->x + s_heading*dist;
    waypoints[target].y = stateGetPositionEnu_f()->y + c_heading*dist;
    
//      struct EnuCoor_f wp_tmp_float_oa[NB_WAYPOINT] = WAYPOINTS_ENU;
//      struct EnuCoor_i wp_tmp_int_oa;
//     
//      wp_tmp_float_oa[target].x = wp_tmp_float_oa[target].x + sinf(angle)*radius;
//      wp_tmp_float_oa[target].y = wp_tmp_float_oa[target].y + cosf(angle)*radius;
//      
//      wp_tmp_int_oa.x = POS_BFP_OF_REAL(wp_tmp_float_oa[target].x);
//      wp_tmp_int_oa.y = POS_BFP_OF_REAL(wp_tmp_float_oa[target].y);
//      wp_tmp_int_oa.z = POS_BFP_OF_REAL(wp_tmp_float_oa[target].z);
//      
// 
//      
//      nav_move_waypoint(target, &wp_tmp_int_oa);
    
    
 
  return FALSE;
}


bool_t NavSetNewWayPoint_13( uint8_t dist, uint8_t target, uint8_t curr)
    {
    int32_t s_heading, c_heading;
    int16_t offset_heading;
    // distance in cm's

    offset_heading = INT32_RAD_OF_DEG((int)(heading_new*360/(2*M_PI)) << (INT32_ANGLE_FRAC));
    PPRZ_ITRIG_SIN(s_heading, stateGetNedToBodyEulers_i()->psi+offset_heading);
    PPRZ_ITRIG_COS(c_heading, stateGetNedToBodyEulers_i()->psi+offset_heading);
    
    waypoints[target].x = waypoints[curr].x + INT_MULT_RSHIFT(dist,s_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;
    waypoints[target].y = waypoints[curr].y + INT_MULT_RSHIFT(dist,c_heading,INT32_TRIG_FRAC-INT32_POS_FRAC) / 100;

    //printf("heading error= %d \n", safe_heading);
    
  return FALSE;
}
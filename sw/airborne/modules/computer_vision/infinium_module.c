/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/practical_module.c
 * @brief Module for the Practical assignment
 *
 * Avoiding obstacles in the arena
 */


#include "infinium_module.h"
#include "generated/flight_plan.h"

#include <stdio.h>
#include <pthread.h>
#include <sys/wait.h>
#include <unistd.h>
#include "state.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"

#include "modules/read_matrix_serial/read_matrix_serial.h"
#include "modules/computer_vision/opticflow_module.h"
//#include "modules/computer_vision/opticflow/stabilization_opticflow.h"

/* default sonar/agl to use in practical visual_estimator */
#ifndef PRACTICAL_AGL_ID
#define PRACTICAL_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(PRACTICAL_AGL_ID);

/* The main variables */
struct practical_t practical;
static struct v4l2_device *practical_video_dev;     //< The main video device
static abi_event practical_agl_ev;                  //< The altitude ABI event
static pthread_t practical_calc_thread;             //< The practical calculation thread

/* Static functions */
static void *practical_module_calc(void *data);                   //< The main calculation thread
static void practical_agl_cb(uint8_t sender_id, float distance);  //< Callback function of the ground altitude
static void practical_tx_img(struct image_t *img, bool_t rtp);  //< Trnsmit an image

static void practical_integral_img_detect(struct image_t *img, uint16_t sub_img_h, uint16_t feature_size, struct image_t* int_y, struct image_t* int_u, struct image_t* int_v);

void nav_cal_heading(float dist_oa, uint8_t goal, uint8_t follow, uint8_t wp_heading);
void nav_cal_heading_stereo(float dist_oa, uint8_t goal, uint8_t follow, uint8_t wp_heading);
uint8_t point_in_sector(struct image_t *img, struct point_t point);
uint16_t num_features_in_sector[4] = {0, 0, 0, 0};

// uint32_t last_second;
// static uint32_t counter;

//color_count_CN
int32_t cnt_obst[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float  b_damp = 5.5; 
float  K_goal = 0;
float  K_obst = 20.0;
float  c1 = 0.4;
float  c2 = 0.4;
float  c3 = 3.0;
float  c5 = 0.9;
float  kv = 0.5;
float  epsilon = 0.1;
float  vmax = 0.5;
float  pos_diff = 2;
uint8_t point_index = 0;
float heading_goal_f = 0;

struct EnuCoor_f waypoints_OA[NB_WAYPOINT] = WAYPOINTS_ENU;

//messages functions
static void send_CNT_OBST(void) {
  DOWNLINK_SEND_CNT_OBST (DefaultChannel, DefaultDevice, cnt_obst);
 }

 static void send_R_DOT_AND_SPEED(void) {
  DOWNLINK_SEND_R_DOT_AND_SPEED (DefaultChannel, DefaultDevice, &r_dot_new, &r_dot_new_sin,  &r_dot_new_cos, &speed_pot);
 }

/*
 * Initialize the practical module
 */
void practical_module_init(void)
{
  //messages
  register_periodic_telemetry(DefaultPeriodic, "CNT_OBST", send_CNT_OBST);
  register_periodic_telemetry(DefaultPeriodic, "R_DOT_AND_SPEED", send_R_DOT_AND_SPEED);
  
   //last_second = get_sys_time_msec();
   //counter = 0;

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(PRACTICAL_AGL_ID, &practical_agl_ev, practical_agl_cb);

  /* Default settings */
#ifdef PRACTICAL_Y_m
  practical.y_m = PRACTICAL_Y_m;
#endif
#ifdef PRACTICAL_Y_M
  practical.y_M = PRACTICAL_Y_M;
#endif
#ifdef PRACTICAL_U_m
  practical.u_m = PRACTICAL_U_m;
#endif
#ifdef PRACTICAL_U_M
  practical.u_M = PRACTICAL_U_M;
#endif
#ifdef PRACTICAL_V_m
  practical.v_m = PRACTICAL_V_m;
#endif
#ifdef PRACTICAL_V_M
  practical.v_M = PRACTICAL_V_M;
#endif
#ifdef PRACTICAL_thres
  practical.trigger_thres = PRACTICAL_thres;
#endif 

  /* Try to initialize the video device */
  practical_video_dev = v4l2_init("/dev/video1", 1280, 720, 10); //TODO: Fix defines
  if (practical_video_dev == NULL) {
    printf("[practical_module] Could not initialize the video device\n");
  }
}

/**
 * Main practical run
 */
void practical_module_run(void)
{

}

/**
 * Start the practical calculation
 */
void practical_module_start(void)
{
  // Check if we are not already running
  if (practical_calc_thread != 0) {
    printf("[practical_module] Calculation of practical already started!\n");
    return;
  }

  // Create the practical calculation thread
  int rc = pthread_create(&practical_calc_thread, NULL, practical_module_calc, NULL);
  if (rc) {
    printf("[practical_module] Could not initialize calculation thread (return code: %d)\n", rc);
  }
  
}

/**
 * Stop the optical flow calculation
 */
void practical_module_stop(void)
{
  // Stop the capturing
  v4l2_stop_capture(practical_video_dev);

  // TODO: fix thread stop
}

   // struct image_t int_y, int_u, int_v;  

    /*#if PRACTICAL_DEBUG
      // Create a new JPEG image
      struct image_t img_jpeg, img_small;
      image_create(&img_jpeg, practical_video_dev->w, practical_video_dev->h, IMAGE_JPEG);
      image_create(&img_small,
	practical_video_dev->w/4,
	practical_video_dev->h/4,
	IMAGE_YUV422);
    #endif */
  
   //uint16_t img_height = 200;
   //   // Try to fetch an image    
    // counter++;
    // if ((get_sys_time_msec()-last_second) > 1000) {
    //   printf("Count: %d\n", counter);
    //   counter = 0;
    //   last_second = get_sys_time_msec();
    // }

    // Try to fetch an image
    //struct image_t img;
    //v4l2_image_get(practical_video_dev, &img);
    
   /* if(int_y.buf_size != 2*img.w*img_height){
      //printf("Not the same size! %d, %d\n", int_y.buf_size, img.w*img_height);
      image_free(&int_y);
      image_free(&int_u);
      image_free(&int_v);
      
      image_create(&int_y, img.w/2, img_height, IMAGE_INTEGRAL);
      image_create(&int_u, img.w/2, img_height, IMAGE_INTEGRAL);
      image_create(&int_v, img.w/2, img_height, IMAGE_INTEGRAL);
    }*/

    // Calculate the colours in 2 bins (left/right)
    // uint32_t bins[2];
    // memset(bins, 0, sizeof(uint32_t) * 2);
    //  image_yuv422_colorfilt(&img, &img_copy, bins, 2, practical.y_m, practical.y_M, practical.u_m, practical.u_M, practical.v_m, practical.v_M);
    // RunOnceEvery(10, printf("Bins: %d\t%d\n", bins[0], bins[1]));

    // // Update the heading
    // if(bins[0] > 50000 || bins[1] > 50000) {
    //   if(bins[1] < bins[0]) {
    //     // Turn left
    //     printf("Turn left.. %d\t%d\n", bins[0], bins[1]);
    //     stabilization_practical_turn(-1);
    //   }
    //   else {
    //     // Turn left
    //     printf("Turn right.. %d\t%d\n", bins[0], bins[1]);
    //     stabilization_practical_turn(1);
    //   }
    // } else {
    //   stabilization_practical_turn(0);
    // }

    // window_h = f(height,pitch, target obstacle avoidacne distance)
    
    // practical_integral_img_detect(&img, img_height /*window_h*/, 25 /*box size*/, &int_y, &int_u, &int_y);

/*#if PRACTICAL_DEBUG
    //RunOnceEvery(10, {
    image_yuv422_downsample(&img, &img_small, 4);
    jpeg_encode_image(&img_small, &img_jpeg, 60, FALSE);
    practical_tx_img(&img_jpeg, FALSE);
    //});
#endif */

    // Free the image
    //v4l2_image_free(practical_video_dev, &img);

  //image_free(&int_y);
  //image_free(&int_u);
  //image_free(&int_v);
  
/*#if PRACTICAL_DEBUG
  image_free(&img_jpeg);
  image_free(&img_small);
#endif
*/


/**
 * Do the main calculation
 */
static void *practical_module_calc(void *data __attribute__((unused)))
{  
  bool_t stereo_flag = 0;
  
  // Start the streaming on the V4L2 device
  if (!v4l2_start_capture(practical_video_dev)) {
    printf("[practical_module] Could not start capture of the camera \n");
    return 0;
  }
  
  struct image_t img;
  struct image_t img_small;
	        
  /* Main loop of the optical flow calculation */
  while (TRUE) {
    
     if(stereo_flag==0){

	    v4l2_image_get(practical_video_dev, &img);
	  // image_yuv422_downsample(&img, &img_small, 4);
	    
	    image_yuv422_colorfilt_CN(&img, &img, practical.y_m, practical.y_M, practical.u_m, practical.u_M, practical.v_m, practical.v_M, cnt_obst);
	    nav_cal_heading(1.0,WP_p1, WP_p2, WP_p3);
	    
	    v4l2_image_free(practical_video_dev, &img);
	    image_free(&img_small);
      }	
      else if(stereo_flag==1){
	    
	    nav_cal_heading_stereo(1.0,WP_p1, WP_p2, WP_p3);
	
      }
  } 
  
}

void nav_cal_heading(float dist_oa, uint8_t goal, uint8_t follow, uint8_t wp_heading){
  bool_t flag_speed_control = FALSE;
  
  if(opticflow_result.vel_x<0.05 && opticflow_result.vel_y<0.05){
    current_heading = 0;
  }
  else if(opticflow_result.vel_x<0.05 && opticflow_result.vel_y>0.05){
    current_heading = 0.5*M_PI;
  }
  else if(opticflow_result.vel_x>0.05 && opticflow_result.vel_y<0.05){
    current_heading = 0; 
  }
  else if(opticflow_result.vel_x>0.05 && opticflow_result.vel_y>0.05){
    current_heading = atan2(opticflow_result.vel_x,opticflow_result.vel_y);
  }  

  heading_goal_ref = current_heading-heading_goal_f;
  
  if (heading_goal_ref>M_PI){
    heading_goal_ref = heading_goal_ref - 2*M_PI;
  }
  else if(heading_goal_ref<-M_PI){
      heading_goal_ref = heading_goal_ref + 2*M_PI;
  }
  
  //define image size HAS TO BE CHECKED!
 int image_size[2] = {1280, 720};
 float image_fow[2] = {1.2915, 2.0297};
 float angle_change;
 
 //define potential field variables
 //float r_new;
 float r_old = stateGetBodyRates_f()->r;

 float potential_obst = 0; 
 float potential_obst_integrated = 0;
 
 //define delta t for integration! check response under 0.1 and 1
 float dt = 0.5;
 
 //calculate position angle and angular widht of obstacles
  for(int i=0;i<5;i++){
   obst_width[i] = cnt_obst[i]*(image_fow[0]/(float)image_size[0]);
   obst_angle[i] = ((float)cnt_obst[10+i]+0.5*(float)cnt_obst[i])*(image_fow[0]/(float)image_size[0])-(0.5*image_fow[0]); 
   
      if (cnt_obst[i]>50 && obst_angle[i] != -(0.5*image_fow[0])){
	potential_obst = potential_obst + K_obst*(-obst_angle[i])*exp(-c3*abs(obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
	potential_obst_write = potential_obst;
	potential_obst_integrated = potential_obst_integrated + K_obst*c3*(abs(obst_angle[i])+1)/(c3*c3)*exp(-c3*abs(obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
	
      }
  }
  
 //calculate angular accelaration from potential field
  r_dot_new = -b_damp*r_old - K_goal*(heading_goal_ref)*(exp(-c1*pos_diff)+c2) + potential_obst;
  r_dot_new_cos = cos(r_dot_new);
  r_dot_new_sin = sin(r_dot_new);
  //r_dot_new =-b_damp*r_old + potential_obst;
  //r_dot_new = heading_goal_ref;
  
  //delta_heading = 0.5*r_dot_new*dt*dt;
  //Integrate using simple intgration CHECK for time step!
  //heading_new = current_heading + 0.5*r_dot_new*dt*dt;

  if(flag_speed_control){
    speed_pot = vmax*exp(-kv*potential_obst_integrated) - epsilon;
    if(speed_pot>0){
	dist_oa = speed_pot*dt;
    }
    else if(speed_pot<=0){
       dist_oa = 0;
       speed_pot = 0;
    }
  }
  
}

void nav_cal_heading_stereo(float dist_oa, uint8_t goal, uint8_t follow, uint8_t wp_heading){
  bool_t flag_speed_control = FALSE;
  
  current_heading = atan(opticflow_result.vel_y/opticflow_result.vel_x);
  heading_goal_ref = current_heading-heading_goal_f;
  
  if (heading_goal_ref>M_PI){
    heading_goal_ref = heading_goal_ref - 2*M_PI;
  }
  else if(heading_goal_ref<-M_PI){
      heading_goal_ref = heading_goal_ref + 2*M_PI;
  }
  
 uint8_t matrix_read[16];
  
  for (int i_m=0;i_m<16;i_m++){
    matrix_read[i_m] = READimageBuffer[i_m];
  }
  
 //define image size HAS TO BE CHECKED!
 int stereo_size[2] = {1, 1};
 int size_matrix[3] = {1, 4, 4};
 float stereo_fow[2] = {0.837758041, 0.628318531};//based on FOW of 48, by 38
 float angle_change;
 float r_o;
 float r_r;
 
 //define potential field variables
 //float r_new;
 float r_old = stateGetBodyRates_f()->r;

 float potential_obst = 0; 
 float potential_obst_integrated = 0;
 
 //define delta t for integration! check response under 0.1 and 1
 float dt = 0.5;
 
 //calculate position angle and angular widht of matrix
 for(int i_o=0;i_o<size_matrix[2];i_o++){ 
      obst_width[i_o] = stereo_fow[0]/size_matrix[2];
      obst_angle[i_o] = - 0.5*stereo_fow[0] - stereo_fow[0]/size_matrix[2] + stereo_fow[0]/size_matrix[2]/2 + (stereo_fow[0]/size_matrix[2])*i_o;
  
}
 
 //calculate position angle and angular widht of obstacles
  for(int i=0;i<size_matrix[2];i++){
       r_o = (tan(obst_angle[i]+0.5*obst_width[i])*(matrix_read[4+i]+matrix_read[8+i])/2) - (tan(obst_angle[i]-0.5*obst_width[i])*(matrix_read[4+i]+matrix_read[8+i])/2);
       r_r = 0.3;
       c5 = M_PI/2 - 2*atan(r_o/(r_o+r_r));
   
      if (cnt_obst[i]>50 && obst_angle[i] != -(0.5*stereo_fow[0])){
	potential_obst = potential_obst + K_obst*(-obst_angle[i])*exp(-c3*abs(obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
	potential_obst_write = potential_obst;
	potential_obst_integrated = potential_obst_integrated + K_obst*c3*(abs(obst_angle[i])+1)/(c3*c3)*exp(-c3*abs(obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
	
      }
  }
  
 //calculate angular accelaration from potential field
  r_dot_new = -b_damp*r_old - K_goal*(heading_goal_ref)*(exp(-c1*pos_diff)+c2) + potential_obst;
  //delta_heading = 0.5*r_dot_new*dt*dt;
  //Integrate using simple intgration CHECK for time step!
  //heading_new = current_heading + 0.5*r_dot_new*dt*dt;

  if(flag_speed_control=FALSE){
    speed_pot = vmax*exp(-kv*potential_obst_integrated) - epsilon;
    if(speed_pot>0){
	dist_oa = speed_pot*dt;
    }
    else if(speed_pot<=0){
       dist_oa = 0;
       speed_pot = 0;
    }
  }
  

  /*waypoints_OA[follow].x = stateGetPositionEnu_f()->x + sinf(heading_new)*dist_oa;
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
 nav_move_waypoint(wp_heading, &wp_tmp_int_oa2);*/
  
}

bool_t nav_cal_heading_vector(float dist_oa, uint8_t goal, uint8_t follow, uint8_t wp_heading){
  bool_t flag_speed_control = FALSE;
  
  current_heading = atan(opticflow_result.vel_y/opticflow_result.vel_x);
  heading_goal_ref = current_heading-heading_goal_f;
  
  if (heading_goal_ref>M_PI){
    heading_goal_ref = heading_goal_ref - 2*M_PI;
  }
  else if(heading_goal_ref<-M_PI){
      heading_goal_ref = heading_goal_ref + 2*M_PI;
  }
  
  //Calculate side slip TODO
  float side_slip = 0.0;
  
  //define image size + FOW STEREO camera, has to be checked!! TODO
 int stereo_size[2] = {1, 1};
 float stereo_fow[2] = {0.6981, 0.6981}; // 40 deg
 int size_matrix[3] = {1, 4, 4};
 
 
 //read in values from STEREO board: 
float StereoInput[1][4][4] = {{{20, 20, 0.1, 20},
			   {20, 20, 0.1, 20},
			   {20, 20, 0.1, 20},
			   {20, 20, 0.1, 20}}};
			   
//Repulsion force, Attracforce and total force with goal
struct FloatVect3 Repulsionforce_Kan = {0,0,0};
struct FloatVect3 Total_Kan = {0,0,0};
struct FloatVect3 Attractforce_goal = {0,0,0};

//Calculate Attractforce_goal size = 1; 
Attractforce_goal.x = 1*cos(heading_goal_f);
Attractforce_goal.y = 1*sin(heading_goal_f);
Attractforce_goal.z = 0;

//Variables needed for Holenstein
float Cfreq = 5.0;  
float Ca = 0.0;
float Cv = 0.0;
float Ko = 1.0;
float Kg = 1.0;
   
 
 //init angle values in input matrix 
float angle_hor = current_heading - 0.5*stereo_fow[0] - stereo_fow[0]/size_matrix[2] + stereo_fow[0]/size_matrix[2]/2;
float angle_ver = 0;
 
for (int i1=0;i1<size_matrix[0];i1++){
    for (int i3=0;i3<size_matrix[2];i3++){
        angle_hor = angle_hor + stereo_fow[0]/size_matrix[2];
        angle_ver = 0.5*stereo_fow[1] + stereo_fow[1]/size_matrix[1] - stereo_fow[1]/size_matrix[1]/2;
	for (int i2=0;i2<size_matrix[1];i2++){
	     angle_ver = angle_ver - stereo_fow[1]/size_matrix[1];
            
	     //Method Kandil
	     if(angle_hor*Cfreq<0.5*M_PI){
	        Ca = cos((angle_hor-(current_heading+side_slip))*Cfreq);
	     }
	     else{
	        Ca = 0;
	     }
	     // Get speed in direction of angle_hor/angle_ver TODO
	     //Cv = F1 + F2 * v/vmax;
	     Cv = 1;
	     if(pow(Cv*Ca/(StereoInput[i1][i2][i3]-0.2),2)<1000){
	        Repulsionforce_Kan.x = Repulsionforce_Kan.x - pow(Cv*Ca/(StereoInput[i1][i2][i3]-0.2),2)*sin(angle_hor)*cos(angle_ver);
		//Repulsionforce_Kan.y = Repulsionforce_Kan.y - Cv*Ca/(StereoInput[i1][i2][i3]-0.2)^2*cos(angle_hor)*cos(angle_ver);
		//Repulsionforce_Kan.z = Repulsionforce_Kan.z - Cv*Ca/(StereoInput[i1][i2][i3]-0.2)^2*sin(angle_ver);
	     }
	     else{
	       	Repulsionforce_Kan.x = Repulsionforce_Kan.x - 1000*sin(angle_hor)*cos(angle_ver);
		//Repulsionforce_Kan.y = Repulsionforce_Kan.y - 1000*cos(angle_hor)*cos(angle_ver);
		//Repulsionforce_Kan.z = Repulsionforce_Kan.z - 1000*sin(angle_ver);
	     }
	}
    }
}

//Normalize for ammount entries in Matrix
//Repulsionforce_Kan = Repulsionforce_Kan/(float)(size_matrix[1]*size_matrix[2]);
VECT3_SMUL(Repulsionforce_Kan, Repulsionforce_Kan, 1.0/(size_matrix[1]*size_matrix[2])); 

//Repulsionforce_Kan_write[0] = (float)Repulsionforce_Kan.x;
//Repulsionforce_Kan_write[1] = (float)Repulsionforce_Kan.y;
//Repulsionforce_Kan_write[2] = (float)Repulsionforce_Kan.z;

VECT3_SMUL(Repulsionforce_Kan,Repulsionforce_Kan,Ko);
VECT3_SMUL(Attractforce_goal, Attractforce_goal, Kg);


//Total force 
VECT3_ADD(Total_Kan, Repulsionforce_Kan);
VECT3_ADD(Total_Kan, Attractforce_goal);

//TODO set correct output 

//Total_Kan_write[0] = (float)Total_Kan.x;
//Total_Kan_write[1] = (float)Total_Kan.y;
//Total_Kan_write[2] = (float)Total_Kan.z;

//  float angle_change;
//  
//  //define potential field variables
//  //float r_new;
//  float r_old = stateGetBodyRates_f()->r;
// 
//  float potential_obst = 0; 
//  float potential_obst_integrated = 0;
//  float speed_pot;
//  
//  //define delta t for integration! check response under 0.1 and 1
//  float dt = 0.5;
//  
//   //calculate position angle and angular widht of obstacles
//   for(int i=0;i<5;i++){
//   obst_width[i] = cnt_obst[i]*(image_fow[0]/(float)image_size[0]);
//   obst_angle[i] = ((float)cnt_obst[10+i]+0.5*(float)cnt_obst[i])*(image_fow[0]/(float)image_size[0])-(0.5*image_fow[0]); 
//   
//       if (cnt_obst[i]>10 && obst_angle[i] != -(0.5*image_fow[0])){
// 	potential_obst = potential_obst + K_obst*(-obst_angle[i])*exp(-c3*abs(obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
// 	potential_obst_write = potential_obst;
// 	potential_obst_integrated = potential_obst_integrated + K_obst*c3*(abs(obst_angle[i])+1)/(c3*c3)*exp(-c3*abs(obst_angle[i]))*(tan(obst_width[i]+c5)-tan(c5));
// 	
//       }
//   }
//   
//   //calculate angular accelaration from potential field
//   r_dot_new = -b_damp*r_old - K_goal*(heading_goal_ref)*(exp(-c1*VECT2_NORM2(pos_diff))+c2) + potential_obst;
//   delta_heading = 0.5*r_dot_new*dt*dt;
//   //Integrate using simple intgration CHECK for time step!
//   heading_new = current_heading + 0.5*r_dot_new*dt*dt;
/*
  
  if(flag_speed_control=TRUE){
    speed_pot = vmax*exp(-kv*potential_obst_integrated) - epsilon;
    if(speed_pot>0){
	dist_oa = speed_pot*dt;
    }
    else if(speed_pot<=0){
       dist_oa = 0;
    }
  }*/
  

 /* waypoints_OA[follow].x = stateGetPositionEnu_f()->x + sinf(heading_new)*dist_oa;
  waypoints_OA[follow].y = stateGetPositionEnu_f()->y + cosf(heading_new)*dist_oa;
  
  waypoints_OA[wp_heading].x = stateGetPositionEnu_f()->x + sinf(heading_new)*2;
  waypoints_OA[wp_heading].y = stateGetPositionEnu_f()->y + cosf(heading_new)*2;
  
  wp_tmp_int_oa.x = POS_BFP_OF_REAL(waypoints_OA[follow].x);
  wp_tmp_int_oa.y = POS_BFP_OF_REAL(waypoints_OA[follow].y);
  wp_tmp_int_oa.z = POS_BFP_OF_REAL(waypoints_OA[follow].z);
  
  wp_tmp_int_oa2.x = POS_BFP_OF_REAL(waypoints_OA[wp_heading].x);
  wp_tmp_int_oa2.y = POS_BFP_OF_REAL(waypoints_OA[wp_heading].y);
  wp_tmp_int_oa2.z = POS_BFP_OF_REAL(waypoints_OA[wp_heading].z);
  */ 
  
 //nav_move_waypoint(follow, &wp_tmp_int_oa);
 //nav_move_waypoint(wp_heading, &wp_tmp_int_oa2);
  
 //return FALSE;
  
}

/**
 * Get the altitude above ground of the drone
 * @param[in] sender_id The id that send the ABI message (unused)
 * @param[in] distance The distance above ground level in meters
 */
static void practical_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  // Update the distance if we got a valid measurement
  if (distance > 0) {
    //opticflow_state.agl = distance; TODO
  }
}

/**
 * Transmit a JPEG image trough RTP or netcat
 * @param[in] *img The image to transmit
 * @param[in] use_netcat If we want to transmit over use netcat or RTP
 */
#if PRACTICAL_DEBUG
static void practical_tx_img(struct image_t *img, bool_t use_netcat)
{
  if (!use_netcat) {
    rtp_frame_send(
      &PRACTICAL_UDP_DEV,       // UDP device
      img,
      0,                        // Format 422
      20,                       // Jpeg-Quality
      0,                        // DRI Header
      0                         // 90kHz time increment
    );
    return;
  }

  // Open process to send using netcat (in a fork because sometimes kills itself???)
  pid_t pid = fork();

  if (pid < 0) {
    printf("[practical_module] Could not create netcat fork.\n");
  } else if (pid == 0) {
    // We are the child and want to send the image
    FILE *netcat = popen("nc 192.168.1.3 5000 2>/dev/null", "w");
    if (netcat != NULL) {
      fwrite(img->buf, sizeof(uint8_t), img->buf_size, netcat);
      pclose(netcat); // Ignore output, because it is too much when not connected
    } else {
      printf("[practical_module] Failed to open netcat process.\n");
    }

    // Exit the program since we don't want to continue after transmitting
    exit(0);
  } else {
    // We want to wait until the child is finished
    wait(NULL);
  }
}
#endif

/**
 * detect object based on color difference with floor
 * @param[in] *img The image to transmit
 * @param[in] sub_img_h The height of the bottom of the frame to search
 * @param[in] feature_size The bin size to average for object detection
 */
static void practical_integral_img_detect(struct image_t *img, uint16_t sub_img_h, uint16_t feature_size, struct image_t* int_y, struct image_t* int_u, struct image_t* int_v)
{/*
  // note numbering of channels 0,1,2 -> Y,U,V
  // TODO: optimise computation of median and integral images in image.c
  // TODO: Remove placing point in image in final version, maybe place in a debug define...
  // TODO: Optimse thresholds for each channel (a bit)
  // TODO: Don't predefine intergral image? Done now to not initialise the memory every loop, maybe can be optimsed

  // Calculate the median values
  uint16_t sub_img_start = img->buf_size - img->w * 2 * sub_img_h;     // location of the start of sub image
  uint8_t median_y = median(img, 0, sub_img_start, sub_img_h);
  uint8_t median_u = median(img, 1, sub_img_start, sub_img_h);
  uint8_t median_v = median(img, 2, sub_img_start, sub_img_h);
  //printf("Median values: %d %d %d\n", median_y, median_u, median_v);

  // Get the integral image
  struct point_t start_point = {
    .x = 0,
    .y = img->h - sub_img_h
  };
  image_get_integral(img, int_y, int_u, int_v, &start_point);

  // Show boxes above thresholds
  struct point_t from, to;
  struct point_t midpoint_feature;
  int16_t diff_y, diff_u, diff_v;// y_value;
  uint8_t  sector;
  uint16_t feature_s2 = feature_size * feature_size;
  num_features_in_sector[0] = num_features_in_sector[1]  = num_features_in_sector[2] = num_features_in_sector[3] = 0;
  for (uint16_t x = 0; x <= img->w - feature_size; x += feature_size) {
    for (uint16_t y = 0; y <= sub_img_h - feature_size; y += feature_size) {
      // Set the from and to
      from.x = x/2;
      from.y = y;
      to.x = (x + feature_size - 1)/2;
      to.y = y + feature_size - 1;

      diff_y = 2*image_get_integral_sum(int_y, &from, &to) / feature_s2 - median_y;
      // y_value = image_get_integral_sum(&int_y, &from, &to) / feature_s2;
      //int16_t diff_y = y_value - median_y;

      // Update the x for the U and V values (since we have 2 times less pixels)
      // from.x /= 2;
      // to.x /= 2;

      diff_u = 2*image_get_integral_sum(int_u, &from, &to) / feature_s2 - median_u;
      diff_v = 2*image_get_integral_sum(int_v, &from, &to) / feature_s2 - median_v;

      // printf("Point: %dY(%d) %dU(%d) %dV(%d) \n", diff_y, median_y, diff_u, median_u, diff_v, median_v);
    
      midpoint_feature.x = x + start_point.x + feature_size/2;
      midpoint_feature.y = y + start_point.y + feature_size/2;

      sector = point_in_sector(img, midpoint_feature);
#if PRACTICAL_DEBUG
      // Show points
      from.x = x + start_point.x;
      from.y = y + start_point.y;
      to.x = from.x + feature_size;
      to.y = from.y + feature_size;

      // if((practical.y_m < y_value) && (y_value < practical.y_M) && (sector != 0)) {
      if(((practical.y_m > diff_y) || (diff_y > practical.y_M)) && (sector != 0)) {
        image_draw_line(img, &from, &to);
      }

      from.x = x + start_point.x + feature_size;
      from.y = y + start_point.y;
      to.x = x + start_point.x;
      to.y = y + start_point.y + feature_size;
      if((practical.u_m > diff_u || diff_u > practical.u_M) && sector != 0) {
        image_draw_line(img, &from, &to);
      }
      if((practical.v_m > diff_v || diff_v > practical.v_M) && sector != 0) {
        image_draw_line(img, &from, &to);
      }
#endif

      // compute number of features per sector
      // if((practical.y_m < y_value && y_value < practical.y_M) || (practical.u_m > diff_u || diff_u > practical.u_M) || (practical.v_m > diff_v || diff_v > practical.v_M)) {
      if((practical.y_m > diff_y || diff_y > practical.y_M) || (practical.u_m > diff_u || diff_u > practical.u_M) || (practical.v_m > diff_v || diff_v > practical.v_M)) {
        num_features_in_sector[sector] += 1;
      }

// Display sector
//       if(sector == 2) {
//         image_draw_line(img, &from, &to);
//       }
    }
  }

  //do avoidance
  if(num_features_in_sector[2] < practical.trigger_thres) {
    //go straight
    yaw_rate = 0;
  }
  else {
    //fly with zero velocity
    //turn
//     if(num_features_in_sector[1] > num_features_in_sector[3])
//       yaw_rate = 2;//turn right
//     else
//       yaw_rate = -2;
    //always turn right
    yaw_rate = 2;//turn right
    // printf("Turn...\n");
  }

  DOWNLINK_SEND_OPTIC_AVOID(DefaultChannel, DefaultDevice,
                            &num_features_in_sector[0],
                            &num_features_in_sector[1],
                            &num_features_in_sector[2],
                            &num_features_in_sector[3]); */

}

uint8_t point_in_sector(struct image_t *img, struct point_t point) {

  struct FloatEulers *eulers = stateGetNedToBodyEulers_f();
  uint8_t sector = 0;
  float path_angle = 0.8;//0.6981;
  struct point_t center_point;

  center_point.x = img->w/2;
  center_point.y = img->h/2 + img->h*eulers->theta*1.5; // the '1.5' is the conversion factor

  // look at a fourth under the center point
  float y_line_at_point = center_point.y + img->h/4 + eulers->phi*(img->w/2 - point.x);

  if(point.y > y_line_at_point) {
    float x_line_left_at_point = img->w/2 - tan(path_angle - eulers->phi) * (point.y - center_point.y);
    float x_line_right_at_point = img->w/2 + tan(path_angle + eulers->phi) * (point.y - center_point.y);
    if(point.x < x_line_left_at_point)
      sector = 1;
    else if(point.x < x_line_right_at_point)
      sector = 2;
    else
      sector = 3;
  }

  return sector;
}

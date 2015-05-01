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

// void NAV_Clint_init {}
#include "nav_color.h"

void nav_color_init(void){}

void nav_color_period(void){}

bool_t test_color_count(void){
  int color_treshold = 6000;
  if(color_count2> color_treshold){
    return TRUE;
  }
  else{
    return FALSE;
  }
}

int32_t nav_color_new_heading90(void){
  safe_heading =0.75;  
  return FALSE;
} 

int32_t nav_color_new_heading180(void){
  safe_heading = 0.75; 
  return FALSE;
} 


bool_t NavSetNewWayPoint(uint8_t curr, uint8_t dist, uint8_t next, uint8_t heading, uint8_t glob, uint8_t look,uint8_t fz1,uint8_t fz2,uint8_t fz3,uint8_t fz4)
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
/*
 * Copyright (C) Clint Nous
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/NAV_Clint/nav_color.h"
 * @author Clint Nous
 * navigation functions flightplan
 */

#ifndef nav_color_H
#define nav_color_H

//inludes
#include "std.h"
#include "navigation.h" 
#include "nav_team13.h"
#include "modules/computer_vision/viewvideo.h"


//functions
void nav_color_init(void);
void nav_color_period(void);
bool_t NavSetNewWayPoint_CN(float dist, uint8_t target);
bool_t NavSetNewWayPoint_13( uint8_t dist, uint8_t target, uint8_t curr);
bool_t nav_cal_heading(float dist_oa, uint8_t goal, uint8_t follow, uint8_t wp_heading);
bool_t NavCalNewWaypointCircle(uint8_t N_points, float radius, uint8_t target);

//variables
int safe_heading;
float heading_new;
float current_heading;
float r_dot_new; 
float delta_heading;
float potential_obst_write;
float obst_angle[10];
float obst_width[10];
float b_damp; 
float K_goal;
float K_obst;
float c1;
float c2;
float c3;
float c5;
float heading_goal_f;
float heading_goal_ref;

#endif


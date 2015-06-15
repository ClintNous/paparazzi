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
 * @file modules/computer_vision/practical_module.h
 * @brief Module for the Practical assignment
 *
 * Avoiding obstacles in the arena
 */

#ifndef PRACTICAL_MODULE_H
#define PRACTICAL_MODULE_H

#include "std.h"
#include "modules/read_matrix_serial/read_matrix_serial.h"

struct practical_t {
  int16_t y_m;
  int16_t y_M;
  int16_t u_m;
  int16_t u_M;
  int16_t v_m;
  int16_t v_M;
  uint8_t trigger_thres;
};
extern struct practical_t practical;
extern uint16_t num_features_in_sector[4];

// Module functions
extern void practical_module_init(void);
extern void practical_module_run(void);
extern void practical_module_start(void);
extern void practical_module_stop(void);

//variables CN
int safe_heading;
float heading_new;
float current_heading;
float delta_heading;
float potential_obst_write;
float obst_angle[10];
float obst_width[10];
extern float b_damp; 
extern float K_goal;
extern float K_obst;
extern float c1;
extern float c2;
extern float c3;
extern float c5;
extern float kv;
extern float epsilon;
extern float vmax;
extern int32_t cnt_obst[20];
float heading_goal_f;
float heading_goal_ref;
float r_dot_new_cos;
float r_dot_new_sin;
extern uint16_t matrix_treshold;
extern uint16_t matrix_sum_treshold;
extern float ref_pitch_angle;
extern int size_matrix[3];
extern uint16_t matrix_sum[6];
extern float Q_kal;
extern float R_kal;

#endif /* PRACTICAL_MODULE_H */
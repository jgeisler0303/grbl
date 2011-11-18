/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  Copyright (c) 2011 Jens Geisler

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

#include <inttypes.h>
#include <math.h>       
#include <stdlib.h>
#include <avr/interrupt.h>

#include "planner.h"
#include "nuts_bolts.h"
#include "stepper.h"
#include "settings.h"
#include "config.h"

// The number of linear motions that can be in the plan at any give time
#ifdef __AVR_ATmega328P__
#define BLOCK_BUFFER_SIZE 16
#else
#define BLOCK_BUFFER_SIZE 5
#endif

#define EPS 1.0e-5 // some very small value

static block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
// todo: this implementation of the buffer wasts memory of one full block only to make the distiction between buffer full and buffer empty
static volatile uint8_t block_buffer_head;       // Index of the next block to be pushed
static volatile uint8_t block_buffer_tail;       // Index of the block to process now

static int32_t plan_position[3]; // The current position of the tool in absolute steps
static double previous_target[3];
static double previous_unit_vec[3];     // Unit vector of previous path line segment
static double previous_nominal_speed;   // Nominal speed of previous path line segment



// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}

// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
static double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration) {
  return( (target_rate*target_rate-initial_rate*initial_rate)/(2*acceleration) );
}


/*                        + <- some maximum rate we don't care about
                         /|\
                        / | \                    
                       /  |  + <- final_rate     
                      /   |  |                   
     initial_rate -> +----+--+                   
                          ^  ^                   
                          |  |                   
      intersection_distance  distance                                                                           */
// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)
static double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance) {
  return( (2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/(4*acceleration) );
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
static double max_allowable_speed(double acceleration, double target_velocity, double distance) {
  return( sqrt(target_velocity*target_velocity + 2*acceleration*distance) );
}

/*                             STEPPER RATE DEFINITION                                              
                                     +--------+   <- nominal_rate
                                    /          \                                
    nominal_rate*entry_factor ->   +            \                               
                                   |             + <- nominal_rate*exit_factor  
                                   +-------------+                              
                                       time -->                                 
*/                                                                              
// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.
// This converts the planner parameters to the data required by the stepper controller.
// NOTE: Final rates must be computed in terms of their respective blocks.
static uint8_t calculate_trapezoid_for_block(block_t *block, uint8_t idx, uint32_t final_rate, block_t *next_block, uint32_t next_initial_rate) {
  
  int32_t acceleration_per_minute = block->rate_delta*ACCELERATION_TICKS_PER_SECOND*60.0;
  int32_t accelerate_steps= 
    ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration_per_minute));
  int32_t decelerate_steps= 
    floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration_per_minute));

  // Calculate the size of Plateau of Nominal Rate. 
  int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
  
  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking 
  // in order to reach the final_rate exactly at the end of this block.
  uint8_t nomi_speed_reached= 0;
  if (plateau_steps < 0) {  
    accelerate_steps = ceil(
      intersection_distance(block->initial_rate, final_rate, acceleration_per_minute, block->step_event_count));
    accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
    accelerate_steps = min(accelerate_steps,block->step_event_count);
    plateau_steps = 0;
  }  else
      nomi_speed_reached= 1;

  cli();
  uint8_t block_buffer_tail_hold= block_buffer_tail; // store to avoid reading volatile twice
  uint8_t block_buffer_head_hold= block_buffer_head; // store to avoid reading volatile twice
  uint8_t idx_inside_queue;
  // is the current block inside the queue? if not: the stepper overtook us
  if(block_buffer_head_hold>=block_buffer_tail_hold) idx_inside_queue= idx>=block_buffer_tail_hold && idx<=block_buffer_head_hold;
  else idx_inside_queue= idx<=block_buffer_head_hold || idx>=block_buffer_tail_hold;
  if(idx_inside_queue && (idx!=block_buffer_tail_hold || idx==block_buffer_head_hold || st_accelerating)) {
    block->accelerate_until= accelerate_steps;
    block->decelerate_after= accelerate_steps+plateau_steps;
    block->final_rate= final_rate;
    if(next_block) next_block->initial_rate= next_initial_rate;
    if(accelerate_steps==block->step_event_count) block->max_accel_reached= 1;
    block->nomi_speed_reached= nomi_speed_reached;
    sei();
    return(true);
  } else {
    sei();
    return(false); // this block is currently being processed by the stepper and it already finished accelerating or the stepper is already finished with this block: we can no longer change anything here
  }
}     

void planner_recalculate() {
  uint8_t current_block_idx= block_buffer_head;
  block_t *next_block, *curr_block = &block_buffer[current_block_idx];
  block_t *prev_block;
  uint8_t prev_block_idx= current_block_idx;
  uint8_t next_block_idx;
  double max_exit_speed;
  uint8_t plan_unchanged= 1;
  
  if(current_block_idx!=block_buffer_tail) { // we cannot do anything to only one block
    double next_entry_speed= 0.0;
    // loop backwards to possibly postpone decceleration
    while(current_block_idx!=block_buffer_tail) { // the second block is the one where we start the forward loop
      prev_block_idx= prev_block_index(prev_block_idx);
      prev_block = &block_buffer[prev_block_idx];

      if(prev_block->max_accel_reached) {
          break; // this blocks entry_speed cannot get any higher
      }
      if(curr_block->max_entry_speed>next_entry_speed)
        next_entry_speed= curr_block->max_entry_speed;
      else
        next_entry_speed= min(max_allowable_speed(settings.acceleration, next_entry_speed, curr_block->millimeters), curr_block->max_entry_speed);
      curr_block->entry_speed= next_entry_speed;
      
      current_block_idx= prev_block_idx;
      curr_block= prev_block;
      
      // we only loop to the point where entry_speed of curr_block cannot be touched any more
      if(curr_block->entry_speed>=curr_block->max_entry_speed || curr_block->nomi_speed_reached) {
          break;
      }
    }
    
    // loop forward, adjust exit speed to not exceed max accelleration
    next_block_idx= current_block_idx;
    while(current_block_idx!=block_buffer_head) {
      next_block_idx= next_block_index(next_block_idx);
      next_block = &block_buffer[next_block_idx];
      
      max_exit_speed= max_allowable_speed(settings.acceleration, curr_block->entry_speed, curr_block->millimeters);
      if(max_exit_speed>next_block->entry_speed)
        max_exit_speed= next_block->entry_speed;

      if(calculate_trapezoid_for_block(curr_block, current_block_idx, max_exit_speed*curr_block->rate_factor, next_block, max_exit_speed*next_block->rate_factor)) {
        next_block->entry_speed= max_exit_speed;
        plan_unchanged= 0;
      } else if(!plan_unchanged) { // we started to modifie the plan an then got overtaken by the stepper executing the plan: this is bad
         st_go_idle(); 
         plan_clear_queue();
         // this is a very bad error that should never occur but still should be handled gracefully
         return;
      }
      
      current_block_idx= next_block_idx;
      curr_block= next_block;
    }
  }
  calculate_trapezoid_for_block(curr_block, current_block_idx, MINIMUM_PLANNER_SPEED*curr_block->rate_factor, NULL, 0);
}

void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  clear_vector(plan_position);
  clear_vector_double(previous_unit_vec);
  clear_vector_double(previous_target);
  previous_nominal_speed = 0.0;
}

void plan_clear_queue() {
  block_buffer_tail = block_buffer_head;
}

void plan_discard_current_block() {
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_tail = next_block_index( block_buffer_tail );
  }
}

block_t *plan_get_current_block() {
  if (block_buffer_head == block_buffer_tail) { return(NULL); }
  return(&block_buffer[block_buffer_tail]);
}

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
void plan_buffer_line(double x, double y, double z, double feed_rate, uint8_t invert_feed_rate) {
  // Calculate target position in absolute steps
  int32_t target[3];
  target[X_AXIS] = lround(x*settings.steps_per_mm[X_AXIS]);
  target[Y_AXIS] = lround(y*settings.steps_per_mm[Y_AXIS]);
  target[Z_AXIS] = lround(z*settings.steps_per_mm[Z_AXIS]);     

  // Calculate the buffer head after we push this byte
  uint8_t next_buffer_head= next_block_index(block_buffer_head);  
  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Rest here until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head) { 
    sleep_mode(); 
  }
  
  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];
  block->direction_bits = 0;

  // Number of steps for each axis
  block->steps_x = target[X_AXIS]-plan_position[X_AXIS];
  block->steps_y = target[Y_AXIS]-plan_position[Y_AXIS];
  block->steps_z = target[Z_AXIS]-plan_position[Z_AXIS];
  
  if(block->steps_x<0) { block->steps_x= -block->steps_x; block->direction_bits |= (1<<X_DIRECTION_BIT); }
  if(block->steps_y<0) { block->steps_y= -block->steps_y; block->direction_bits |= (1<<Y_DIRECTION_BIT); }
  if(block->steps_z<0) { block->steps_z= -block->steps_z; block->direction_bits |= (1<<Z_DIRECTION_BIT); }
  block->step_event_count = max(block->steps_x, max(block->steps_y, block->steps_z));
  // Bail if this is a zero-length block
  if (block->step_event_count == 0) { return; };
  

  double delta_mm[3];
  delta_mm[X_AXIS]= x-previous_target[X_AXIS]; previous_target[X_AXIS]= x;
  delta_mm[Y_AXIS]= y-previous_target[Y_AXIS]; previous_target[Y_AXIS]= y;
  delta_mm[Z_AXIS]= z-previous_target[Z_AXIS]; previous_target[Z_AXIS]= z;
  block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + square(delta_mm[Z_AXIS]));
  double inverse_millimeters= 1.0/block->millimeters;
  
  // Calculate speed in mm/minute for each axis. No divide by zero due to previous checks.
  // NOTE: Minimum stepper speed is limited by MINIMUM_STEPS_PER_MINUTE in stepper.c
  double inverse_minute;
  if (!invert_feed_rate) {
    inverse_minute = feed_rate * inverse_millimeters;
  } else {
    inverse_minute = 1.0 / feed_rate;
  }
  block->nominal_speed = block->millimeters * inverse_minute; // (mm/min) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_minute); // (step/min) Always > 0

  block->rate_delta = ceil( block->step_event_count*inverse_millimeters *  
        settings.acceleration / (60 * ACCELERATION_TICKS_PER_SECOND )); // (step/min/acceleration_tick)

  // Compute path unit vector                            
  double unit_vec[3];

  unit_vec[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;
  unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
  unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;  
  
  double vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

  // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
  if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) {
    // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
    // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
    double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS] 
                       - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS] 
                       - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
                         
    // Skip and use default max junction speed for 0 degree acute junction.
    if (cos_theta < 0.95) {
      vmax_junction = min(previous_nominal_speed,block->nominal_speed);
      // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
      if (cos_theta > -0.95) {
        // Compute maximum junction velocity based on maximum acceleration and junction deviation
        double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
        vmax_junction = min(vmax_junction,
          sqrt(settings.acceleration * settings.junction_deviation * sin_theta_d2/(1.0-sin_theta_d2)) );
      }
    }
  }
  block->max_entry_speed = vmax_junction;
  
  block->rate_factor= block->nominal_rate/block->nominal_speed;

  block->entry_speed= MINIMUM_PLANNER_SPEED;
  block->initial_rate= MINIMUM_PLANNER_SPEED*block->rate_factor;
  block->max_accel_reached= 0;

  memcpy(previous_unit_vec, unit_vec, sizeof(unit_vec)); // previous_unit_vec[] = unit_vec[]
  previous_nominal_speed = block->nominal_speed;

  planner_recalculate();
  block_buffer_head = next_buffer_head;

  // Update position 
  memcpy(plan_position, target, sizeof(target)); // plan_position[] = target[]
  st_cycle_start();
}

// set the planner position vector
// called via mc_set_current_position
void plan_set_current_position(double x, double y, double z) {
  plan_position[X_AXIS] = lround(x*settings.steps_per_mm[X_AXIS]);
  plan_position[Y_AXIS] = lround(y*settings.steps_per_mm[Y_AXIS]);
  plan_position[Z_AXIS] = lround(z*settings.steps_per_mm[Z_AXIS]);
  previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
  clear_vector_double(previous_unit_vec);
  previous_target[X_AXIS]= x;
  previous_target[Y_AXIS]= y;
  previous_target[Z_AXIS]= z;
}

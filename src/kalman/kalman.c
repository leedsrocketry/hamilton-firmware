/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 18 May 2025
  Description: Kalman filter for HFC
*/

#include <inttypes.h>
#include <stdlib.h> // For malloc and free
#include <stdio.h>  // For printf
#include <string.h> // For strtok
#include <float.h> 
#include "kalman.h"

void kalman_predict(State *s) {
  s->height += (s->speed * STEP + 0.5f * s->accel * STEP * STEP);
  s->speed += (s->accel * STEP);
}

void kalman_err_height(State *state, Sample *sample, Error *error)
{
    float height_distrust;
    float speed_distrust;

    error->height_error = sample->height - state->height;

    if (state->flight_state >= ao_flight_drogue)
        return;

    // Height distrust: how much are we above the "trustworthy" barometer range
    height_distrust = sample->height - AO_MAX_BARO_HEIGHT;
    
    // Speed distrust: how fast are we going (high speed = less trust in baro)
    // Original: AO_MS_TO_SPEED converts m/s to internal units (* 16)
    // We want m/s, so: AO_MAX_BARO_SPEED is 248 m/s
    speed_distrust = (state->speed - 248.0f) * 2.0f; // *2 accounts for the >>3 shift in original
    
    if (speed_distrust < 0)
        speed_distrust = 0;
    if (speed_distrust > AO_MAX_SPEED_DISTRUST)
        speed_distrust = AO_MAX_SPEED_DISTRUST;
    if (speed_distrust > height_distrust)
        height_distrust = speed_distrust;
        
    if (height_distrust > 0) {
        if (height_distrust > 256.0f)
            height_distrust = 256.0f;
        error->height_error = error->height_error * (256.0f - height_distrust) / 256.0f;
    }
}

// void kalman_err_accel(state *state, sample *sample, error *error)
// {
//     float accel_ms2 = sample->accel * 9.80665f / 1000.0f;
//     float ground_accel_ms2 = ACCEL_PLUS_G_OFFSET * 9.80665f / 1000.0f;

//     error->accel_error = (ground_accel_ms2 - accel_ms2) - state->accel;
// }

void kalman_err_accel(State *state, Sample *sample, Error *error)
{
    float accel_corrected;
    
    accel_corrected = (ACCEL_PLUS_G - sample->accel) * ACCEL_SCALE;
    
    error->accel_error = accel_corrected - state->accel;
}

void kalman_correct(State *state, Error *e) {
  state->height += (e->height_error * K00 + e->accel_error * K01);
  state->speed += (e->height_error * K10 + e->accel_error * K11);
  state->accel += (e->height_error * K20 + e->accel_error * K21);
}

void kalman_filter(Sample *s, State *state) {
  // 1. Predict the next state
  kalman_predict(state);

  // 2. Calculate the errors based on the current measurement
  Error e;
  kalman_err_height(state, s, &e);
  kalman_err_accel(state, s, &e);

  // 3. Correct the state based on the errors
  kalman_correct(state, &e);
}
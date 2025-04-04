/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 26 July 2024
  Description: System for managing flight
*/

#include "flight_manager.h"

#include "buffer.h"
#include "HAL/STM32_init.h"

FlightStage flightStage = LAUNCHPAD;

FlightStage get_flight_stage() { return flightStage; }

void set_flight_stage(FlightStage fs) { flightStage = fs; }

double max_altitude = 0;
double ground_altitude = 0;

typedef struct {
  float velocity_z;
  double altitude;
} State;

State state;

void handle_LAUNCHPAD(Frame *frame) {
  if (state.altitude > (ground_altitude + ALTITUDE_APOGEE_THRESHOLD)) {
    LOG("LAUNCHPAD: Altitude threshold met\r\n");
    flightStage = ASCENT;
    return;
  }

  if (frame->accel.x < ACCEL_LAUNCH_THRESHOLD) {
    LOG("LAUNCHPAD: Acceleration threshold met\r\n");
    flightStage = ASCENT;
    return;
  }
}

void handle_ASCENT(Frame *frame) {
  if (state.altitude > max_altitude) {
    max_altitude = state.altitude;
  }

  if (state.altitude < (max_altitude - ALTITUDE_APOGEE_THRESHOLD)) {
    LOG("ASCENT: Altitude threshold met\r\n");
    flightStage = APOGEE;
  }
}

// Unneeded?
void handle_APOGEE(Frame *frame) {
  STM32_blink_flash();
  flightStage = DESCENT;
  (void)frame;
}

void handle_DESCENT(Frame *frame, CircularBuffer *cb) {
  (void)frame;

  uint32_t range = cb_pressure_range(cb);
  if (range < GROUND_THRESHOLD) {
    LOG("DESCENT: Pressure threshold met\r\n");
    flightStage = LANDING;
  }
}

void handle_LANDING(Frame *frame) {
  STM32_blink_flash();
  (void)frame;
}

void run_flight() {
  init_flash();

  Frame init_frame;
  read_sensors(&init_frame, 33);

  ground_altitude = barometric_equation(init_frame.barometer.pressure, init_frame.barometer.temp);

  CircularBuffer *cb = cb_create(20);

  // for(int i = 0; i < 40; i++) {
  //   Frame frame;
  //   read_sensors(&frame, 33);
  //   (void)cb_enqueue_overwrite(cb, &frame);
  // }

  uint32_t start_time = get_time_ms();
  uint32_t last_loop_time = start_time;  // Initialize last_loop_time
  uint32_t current_time;
  uint32_t dt = 0;

  // State state;
  state.altitude = ground_altitude;
  state.velocity_z = 0;

  Frame frame;
  Frame avg_frame;

  for(uint32_t i = 0; i < 25; i++) {
    read_sensors(&frame, 33);
    (void)cb_enqueue_overwrite(cb, &frame);
  }

  for (;;) {
    current_time = get_time_ms();
    dt = current_time - last_loop_time;
    last_loop_time = current_time;

    read_sensors(&frame, dt);
    int8_t write_success = save_frame(frame);
    if (write_success != SUCCESS) {
      LOG("WRITE FAILED\r\n");
    }

    (void)cb_enqueue_overwrite(cb, &frame);

    (void)cb_average(cb, &avg_frame);
 
    state.altitude = ((ground_altitude) - barometric_equation((double)avg_frame.barometer.pressure, (double)avg_frame.barometer.temp) / 10);

    // printf_float("alt", (float)barometric_equation((double)avg_frame.barometer.pressure, (double)avg_frame.barometer.temp), true);
    // LOG("\r\n");

    switch (flightStage) {
      case LAUNCHPAD:
        handle_LAUNCHPAD(&avg_frame);
        break;
      case ASCENT:
        handle_ASCENT(&avg_frame);
        break;
      case APOGEE:
        handle_APOGEE(&avg_frame);
        break;
      case DESCENT:
        handle_DESCENT(&avg_frame);
        break;
      case LANDING:
        handle_LANDING(&avg_frame);
        break;
      default:
        break;
    }
  }
}
/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 26 July 2024
  Description: System for managing flight
*/

#include "flight_manager.h"

#include "HAL/STM32_init.h"
#include "buffer.h"

uint64_t loop_count = 0;

FlightStage flightStage = LAUNCHPAD;

FlightStage get_flight_stage() { return flightStage; }

void set_flight_stage(FlightStage fs) { flightStage = fs; }

double max_altitude = 0;
double ground_altitude = 0;
uint32_t apogee_time = 0;

typedef struct {
  float velocity_z;
  double altitude;
} State;

State state;

void handle_LAUNCHPAD(Frame *avg_frame, Frame *cur_frame) {
  (void)avg_frame;
  if (state.altitude > ALTITUDE_APOGEE_THRESHOLD) {
    logi("LAUNCHPAD: Altitude threshold met\r\n");
    flightStage = ASCENT;
    return;
  }

  if (cur_frame->accel.y < ACCEL_LAUNCH_THRESHOLD) {
    logi("LAUNCHPAD: Acceleration threshold met\r\n");
    flightStage = ASCENT;
    return;
  }
}

void handle_ASCENT(Frame *frame) {
  (void)frame;
  if (state.altitude > max_altitude) {
    max_altitude = state.altitude;
  }

  if (state.altitude < (max_altitude - ALTITUDE_APOGEE_THRESHOLD)) {
    logi("ASCENT: Altitude threshold met\r\n");
    flightStage = APOGEE;
  }
}

// Unneeded?
void handle_APOGEE(Frame *frame) {
  flightStage = DESCENT;
  apogee_time = get_time_ms();
  (void)frame;
}

void handle_DESCENT(Frame *frame, CircularBuffer *cb) {
  (void)frame;
  (void)cb;

  // uint32_t range = cb_pressure_range(cb);
  // if (range < GROUND_THRESHOLD) {
  //   LOG("DESCENT: Pressure threshold met\r\n");
  //   flightStage = LANDING;
  // }
}

void handle_LANDING(Frame *frame) {
  STM32_blink_flash();
  (void)frame;
}

void run_flight() {
  init_flash();

  Frame init_frame;
  read_sensors(&init_frame, 33);

  CircularBuffer *cb = cb_create(20);

  uint32_t start_time = get_time_ms();
  uint32_t last_loop_time = start_time;  // Initialize last_loop_time
  uint32_t current_time;
  uint32_t dt = 0;

  Frame frame;
  Frame avg_frame;

  for (uint32_t i = 0; i < 25; i++) {
    read_sensors(&frame, 33);  // this DT should probably be calculated. But doesn't REALLY matter.
    (void)cb_enqueue_overwrite(cb, &frame);
    (void)cb_average(cb, &avg_frame);
  }
  (void)cb_average(cb, &avg_frame);
  print_sensor_line(avg_frame);
  ground_altitude = (barometric_equation((double)avg_frame.barometer.pressure, (double)avg_frame.barometer.temp));
  // printf_float("GROUND alt", ground_altitude, true); LOG("\r\n");

  // State state;
  state.altitude = ground_altitude;
  state.velocity_z = 0;

  (void)cb_average(cb, &avg_frame);

  for (;;) {
    current_time = get_time_ms();
    dt = current_time - last_loop_time;
    last_loop_time = current_time;

    if (apogee_time != 0 && (current_time - apogee_time) > 600000) {
      logw("APOGEE TIMEOUT\r\n");
      flightStage = LANDING;
    }

    read_sensors(&frame, dt);
    if (flightStage != LAUNCHPAD && flightStage != LANDING) {
      int8_t write_success = log_frame(frame);
      if (write_success != SUCCESS) {
        loge("WRITE FAILED\r\n");
      }
    }

    (void)log_frame(frame);

    (void)cb_enqueue_overwrite(cb, &frame);

    (void)cb_average(cb, &avg_frame);
    print_sensor_line(avg_frame);

    state.altitude =
        (barometric_equation((double)avg_frame.barometer.pressure, (double)avg_frame.barometer.temp) - ground_altitude);

    switch (flightStage) {
      case LAUNCHPAD:
        // if (loop_count % 10 == 0) {
        //   STM32_beep_buzzer(25, 25, 1);
        // }
        handle_LAUNCHPAD(&avg_frame, &frame);
        break;
      case ASCENT:
        handle_ASCENT(&avg_frame);
        break;
      case APOGEE:
        handle_APOGEE(&avg_frame);
        break;
      case DESCENT:
        handle_DESCENT(&avg_frame, cb);
        break;
      case LANDING:
        handle_LANDING(&avg_frame);
        break;
      default:
        break;
    }

    loop_count++;
  }
}
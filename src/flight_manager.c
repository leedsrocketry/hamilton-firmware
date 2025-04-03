/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 26 July 2024
  Description: System for managing flight
*/

#include "flight_manager.h"

#include "buffer.h"

FlightStage flightStage = LAUNCHPAD;

FlightStage get_flight_stage() { return flightStage; }

void set_flight_stage(FlightStage fs) { flightStage = fs; }

double max_altitude = 0;
double ground_altitude = 0;

void handle_LAUNCHPAD(Frame *frame) {
  double altitude = barometric_equation(frame->barometer.pressure, frame->barometer.temp);
  (void)altitude;
  // printf_float("altitude", (float)altitude, true);
  // printf("\r\n");

  // if (altitude > (ground_altitude + ALTITUDE_APOGEE_THRESHOLD)) {
  //   LOG("LAUNCHPAD: Altitude threshold met\r\n");
  //   flightStage = ASCENT;
  //   return;
  // }

  // if (frame->accel.x < ACCEL_LAUNCH_THRESHOLD) {
  //   LOG("LAUNCHPAD: Acceleration threshold met\r\n");
  //   flightStage = ASCENT;
  //   return;
  // }
}

void handle_ASCENT(Frame *frame) {
  double altitude = barometric_equation(frame->barometer.pressure, frame->barometer.temp);

  printf_float("altitude", (float)altitude, true);
  printf("\r\n");

  if (altitude < (max_altitude - ALTITUDE_APOGEE_THRESHOLD)) {
    LOG("ASCENT: Altitude threshold met\r\n");
    flightStage = DESCENT;
  }
}

// Unneeded?
void handle_APOGEE(Frame *frame) { (void)frame; }

void handle_DESCENT(Frame *frame) { (void)frame; }

void handle_LANDING(Frame *frame) { (void)frame; }

void run_flight() {
  init_flash();

  Frame init_frame;
  read_sensors(&init_frame, 33);

  ground_altitude = barometric_equation(init_frame.barometer.pressure, init_frame.barometer.temp);

  CircularBuffer *cb = cb_create(20);

  uint32_t start_time = get_time_ms();
  uint32_t last_loop_time = start_time;  // Initialize last_loop_time
  uint32_t current_time;
  uint32_t dt = 0;
  (void)dt;

  for (;;) {
    current_time = get_time_ms();
    dt = current_time - last_loop_time;
    last_loop_time = current_time;

    Frame frame;
    read_sensors(&frame, dt);
    int8_t write_success = save_frame(frame);
    if (write_success != SUCCESS) {
      LOG("WRITE FAILED\r\n");
    }

    (void)cb_enqueue_overwrite(cb, &frame);
    Frame avg_frame;

    (void)cb_average(cb, &avg_frame);
    print_sensor_line(frame);

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
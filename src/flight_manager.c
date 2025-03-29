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

void handle_LAUNCHPAD(Frame *frame) {
  if (frame->accel.x < ACCEL_LAUNCH_THRESHOLD) {
    LOG("LAUNCHPAD: Acceleration threshold met\r\n");
    flightStage = ASCENT;
  }
}

void handle_ASCENT(Frame *frame) {
  double altitude =
      barometric_equation(frame->barometer.pressure, frame->barometer.temp);
  
  if (altitude < (max_altitude-ALTITUDE_APOGEE_THRESHOLD)) {
    LOG("LAUNCHPAD: Acceleration threshold met\r\n");
    flightStage = DESCENT;
  }

}

// Unneeded?
void handle_APOGEE(Frame *frame) { (void)frame; }

void handle_DESCENT(Frame *frame) {
  (void)frame;
}

void handle_LANDING(Frame *frame) { (void)frame; }

void run_flight() {
  init_flash();

  CircularBuffer *cb = cb_create(20);

  for (;;) {
    Frame frame;
    read_sensors(&frame);
    int8_t write_success = log_frame(frame);
    if (write_success != SUCCESS) {
      LOG("WRITE FAILED\r\n");
    }

    cb_enqueue_overwrite(cb, &frame);
    Frame avg_frame;
    cb_average(cb, &avg_frame);
    print_sensor_line(avg_frame);

    // LOG("Flight stage: %d\r\n", flightStage);
    switch (flightStage) {
      case LAUNCHPAD:
        handle_LAUNCHPAD(&avg_frame);
        break;
      case ASCENT:
        handle_ASCENT(&frame);
        break;
      case APOGEE:
        handle_APOGEE(&frame);
        break;
      case DESCENT:
        handle_DESCENT(&frame);
        break;
      case LANDING:
        handle_LANDING(&frame);
        break;
      default:
        break;
    }
  }
}
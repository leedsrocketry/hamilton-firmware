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

#define to_fix_v(x) ((int16_t) ((x) * 65536.0 + 0.5))
#define to_fix_k(x) ((int32_t) ((x) * 65536.0 + 0.5))

// Absolute magic numbers stolen from the altos nickle code.
#define K00 (0.0249599159)
#define K01 (0.0042974476)
#define K10 (0.0102089813)
#define K11 (0.0282214935)
#define K20 (0.0004774942)
#define K21 (0.1612420179)

#define STEP (0.032)
#define STEP_100    (0.01)
#define STEP_2_2_100  (0.00005)

#define STEP_10   (0.1)
#define STEP_2_2_10 (0.005)

#define STEP_1    (1)
#define STEP_2_2_1    (0.5)

#define AO_MAX_BARO_HEIGHT  30000

#define AO_MAX_BARO_SPEED 248

#define AO_MAX_SPEED_DISTRUST 160

#ifndef AO_MS_TO_SPEED
#define AO_MS_TO_SPEED(ms) ((ms) * 16.0f)
#endif

#ifndef ACCEL_SCALE
#define ACCEL_SCALE (1000.0f)
#endif

#ifndef ACCEL_PLUS_G_OFFSET
#define ACCEL_PLUS_G_OFFSET (-1000.0f)
#endif

struct state {
  float height;
  float speed;
  float accel;
  enum flight_state {
    ao_flight_drogue,
    ao_flight_main,
    ao_flight_landed,
    ao_flight_impact
  } flight_state;
} typedef state;

struct error {
  float height_error;
  float accel_error;
} typedef error;

struct sample {
  float height;
  float speed;
  float accel;
} typedef sample;

void kalman_predict(state *s) {
  s->height += (s->speed * STEP + 0.5f * s->accel * STEP * STEP);
  s->speed += (s->accel * STEP);
}


void ao_kalman_err_height(state *state, sample *sample, error *error)
{
    float  e;
    float height_distrust;
    float  speed_distrust;

    error->height_error = sample->height - state->height;

    // e = ao_error_h;
    // if (e < 0)
    //     e = -e;
    // if (e > 127)
    //     e = 127;
    // ao_error_h_sq_avg -= ao_error_h_sq_avg / 16.0f;
    // ao_error_h_sq_avg += (e * e) / 16.0f;

    if (state->flight_state >= ao_flight_drogue)
        return;

    height_distrust = sample->height - AO_MAX_BARO_HEIGHT;
    speed_distrust = (state->speed - (float)(AO_MAX_BARO_SPEED)) / 8.0f;
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

void ao_kalman_err_accel(state *state, sample *sample, error *error)
{
    float accel_ms2 = sample->accel * 9.80665f / 1000.0f; // Convert milligs to m/s^2
    float ground_accel_ms2 = ACCEL_PLUS_G_OFFSET * 9.80665f / 1000.0f; // Convert offset to m/s^2

    error->accel_error = (ground_accel_ms2 - accel_ms2) - state->accel;
}

void kalman_correct(state *state, error *e) {
  state->height += (e->height_error * K00 + e->accel_error * K01);
  state->speed += (e->height_error * K10 + e->accel_error * K11);
  state->accel += (e->height_error * K20 + e->accel_error * K21);
}

void kalman_filter(sample *s, state *state) {
  // 1. Predict the next state
  kalman_predict(state);

  // 2. Calculate the errors based on the current measurement
  error e;
  ao_kalman_err_height(state, s, &e);
  ao_kalman_err_accel(state, s, &e);

  // 3. Correct the state based on the errors
  kalman_correct(state, &e);
}

// Example standalone main function to demonstrate usage.
// LLM Generated.
int main() {
  state current_state = {0.0f, 0.0f, 0.0f}; // Initial state
  sample current_sample;
  FILE *fp_in;
  char line[256];
  const char delimiter[] = ",";
  char *token;
  float accel_y, height;

  fp_in = fopen("data.csv", "r");
  if (fp_in == NULL) {
    perror("Error opening data.csv for reading");
    return 1;
  }

  float max_height = -FLT_MAX; // Initialize to a very small value
  float max_speed = -FLT_MAX;
  float max_accel = -FLT_MAX;

  printf("Time,Estimated_Height,Estimated_Speed,Estimated_Accel\n");

  if (fgets(line, sizeof(line), fp_in) == NULL) {
    fprintf(stderr, "Error reading header line from data.csv or file is empty.\n");
    fclose(fp_in);
    return 1;
  }

  int time_step = 0;
  while (fgets(line, sizeof(line), fp_in) != NULL) {
    token = strtok(line, delimiter);
    if (token != NULL) {
      accel_y = atof(token);
      token = strtok(NULL, delimiter);
      if (token != NULL) {
        height = atof(token);

        current_sample.accel = accel_y;
        current_sample.height = height;

        kalman_filter(&current_sample, &current_state);

        if (current_state.height > max_height) {
          max_height = current_state.height;
        }
        if (current_state.speed > max_speed) {
          max_speed = current_state.speed;
        }
        if (current_state.accel > max_accel) {
          max_accel = current_state.accel;
        }

        printf("%d,%.6f,%.6f,%.6f\n",
               time_step++, current_state.height, current_state.speed, current_state.accel);
      } else {
        fprintf(stderr, "Error parsing height from line: %s", line);
      }
    } else {
      fprintf(stderr, "Error parsing accel_y from line: %s", line);
    }
  }

  fclose(fp_in);

  // Print the maximum values
  printf("\nMaximum Values:\n");
  printf("Max Height: %.6f\n", max_height);
  printf("Max Speed: %.6f\n", max_speed);
  printf("Max Accel: %.6f\n", max_accel);

  return 0;
}

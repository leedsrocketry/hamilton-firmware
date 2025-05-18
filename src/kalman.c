/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 18 May 2025
  Description: Kalman filter for HFC
*/

#include <inttypes.h>

#define to_fix_v(x) ((int16_t) ((x) * 65536.0 + 0.5))
#define to_fix_k(x) ((int32_t) ((x) * 65536.0 + 0.5))

// Should be converted to fixed point later
#define BOTH_K00_100 (0.0081270401)
#define BOTH_K01_100 (0.0000560254)
#define BOTH_K10_100 (0.0531264687)
#define BOTH_K11_100 (0.0094928269)
#define BOTH_K20_100 (0.0015936115)
#define BOTH_K21_100 (0.0951244252)

#define STEP_100		(0.01)
#define STEP_2_2_100	(0.00005)

#define STEP_10		(0.1)
#define STEP_2_2_10	(0.005)

#define STEP_1		(1)
#define STEP_2_2_1		(0.5)

#define AO_MAX_BARO_HEIGHT	30000

/*
 * Above this speed, baro measurements are unreliable
 */
#define AO_MAX_BARO_SPEED	248

/* The maximum amount (in a range of 0-256) to de-rate the
 * baro sensor data based on speed.
 */
#define AO_MAX_SPEED_DISTRUST	160

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
  s->height += (s->speed * STEP_100 + s->accel * STEP_2_2_100);
  s->speed += (s->accel * STEP_100);
}

void ao_kalman_err_height(state *state, sample *sample, error *error)
{
    float  e;
    float height_distrust;
    float  speed_distrust;

    // ao_error_h = ao_sample_height - (float) (ao_k_height >> 16);

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
    /* speed is stored * 16, but we need to ramp between 248 and 328, so
     * we want to multiply by 2. The result is a shift by 3.
     */
    speed_distrust = (state->speed - (float)AO_MS_TO_SPEED(AO_MAX_BARO_SPEED)) / 8.0f;
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

// void ao_kalman_err_accel(void)
// {
//     int32_t  accel;

//     accel = (ao_config.accel_plus_g - ao_sample_accel) * ao_accel_scale;

//     /* Can't use ao_accel here as it is the pre-prediction value still */
//     ao_error_a = (float) ((accel - ao_k_accel) >> 16);
// }


void kalman_correct(state *state) {

  // Create error and sample structs
  error *e = (error *)malloc(sizeof(error));
  sample *s = (sample *)malloc(sizeof(sample));

  kalman_height_error(state, s, e);

  state->height += (e->height_error * BOTH_K00_100 + e->accel_error * BOTH_K01_100);
  state->speed += (e->height_error * BOTH_K10_100 + e->accel_error * BOTH_K11_100);
  state->accel += (e->height_error * BOTH_K20_100 + e->accel_error * BOTH_K21_100);
}

void kalman(sample *s, state *state) {
  // Predict the next state
  kalman_predict(state);

  // Correct the state with the new sample
  kalman_correct(state);

  // Free allocated memory
  free(s);
  free(state);
}
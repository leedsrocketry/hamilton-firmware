#define to_fix_v(x) ((int16_t) ((x) * 65536.0 + 0.5))
#define to_fix_k(x) ((int32_t) ((x) * 65536.0 + 0.5))

// Absolute magic numbers stolen from the altos nickle code.
#define K00 (0.0161816451)
#define K01 (0.0023369070)
#define K10 (0.0066148600)
#define K11 (0.0185629934)
#define K20 (0.0002596563)
#define K21 (0.1317719634)

#define STEP (0.023)
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

#define ACCEL_PLUS_G (1000.0f)

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
} typedef State;

struct error {
  float height_error;
  float accel_error;
} typedef Error;

struct sample {
  float height;
  float speed;
  float accel;
} typedef Sample;

void kalman_filter(Sample *s, State *state);
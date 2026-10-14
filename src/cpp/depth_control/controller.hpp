#pragma once

#define MIN_OUTPUT -1.0
#define MAX_OUTPUT 1.0

#define EMA_Alpha 0.5 // 0 < \alpha < 1

typedef struct {
  double k_p;
  double k_i;
  double k_d;
} Weights;

class Controller {
public:
  // pid weights
  Weights weights;

  // depth target
  double target = 0;

  // pid values
  double integral = 0;
  double last_error = 0;

  // exponential moving average filter
  double moving_avg = 0;

  int num_items = 0;

  Controller(Weights w) : weights(w) {};
  double compute(double measurement, double dt);
  double compute_filtered(double dt);
  void record_measurement(double measurement);
  double get_measurement();
};

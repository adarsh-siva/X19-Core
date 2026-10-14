#include "controller.hpp"

#define MAX(a, b)

double Controller::compute(double measurement, double dt) {
  double error = target - measurement;
  double p = weights.k_p;

  integral += error * dt;
  double i = weights.k_i * integral;

  double derivative = (error - last_error) / dt;
  double d = weights.k_d * derivative;

  double output = p + i + d;

  if (output > MAX_OUTPUT) {
    output = MAX_OUTPUT;
    integral -= error * dt;
  } else if (output < MIN_OUTPUT) {
    output = MIN_OUTPUT;
    integral -= error * dt;
  }

  last_error = error;

  return output;
}

double Controller::compute_filtered(double dt) {
  return this->compute(moving_avg, dt);
}

void Controller::record_measurement(double measurement) {
  // s_t     = \alpha    * x_t         + (1 - \alpha)    * s_{t-1}
  moving_avg = EMA_Alpha * measurement + (1 - EMA_Alpha) * moving_avg;
}

double Controller::get_measurement() {
  return moving_avg;
}

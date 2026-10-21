#include "controller.hpp"

double Controller::compute(double measurement, double dt) {
  double error = target - measurement;
  double p = weights.k_p * error;

  integral += error * dt;
  double i = weights.k_i * integral;

  // no previous error on the first call: start the derivative at 0 instead
  // of kicking with (error - 0) / dt
  if (num_items == 0) {
    last_error = error;
  }
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
  num_items++;

  return output;
}

double Controller::compute_filtered(double dt) {
  return this->compute(moving_avg, dt);
}

void Controller::record_measurement(double measurement) {
  // seed the filter with the first sample; starting from 0 would read as
  // "at the surface" for the first several samples
  if (!has_measurement) {
    moving_avg = measurement;
    has_measurement = true;
    return;
  }
  // s_t     = \alpha    * x_t         + (1 - \alpha)    * s_{t-1}
  moving_avg = EMA_Alpha * measurement + (1 - EMA_Alpha) * moving_avg;
}

double Controller::get_measurement() {
  return moving_avg;
}

void Controller::reset() {
  integral = 0;
  last_error = 0;
  moving_avg = 0;
  num_items = 0;
  has_measurement = false;
}

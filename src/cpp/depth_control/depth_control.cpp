#include "controller.hpp"
#include "depth_control_subscriber.hpp"

double read_depth() {
  return 5;
}

void send_thruster_power(double value);

int main() {
  Controller *controller = new Controller((Weights) {.k_p = 0, .k_i = 0, .k_d = 0});
  DepthControlSubscriber sub;

  sub.spinOnce();

  while (sub.getEnableControl()) {
    // recieve protobuf message
    sub.spinOnce();
    double target = sub.getTargetDepth();

    // recieve sensor data
    double measurement = read_depth();
    controller->record_measurement(measurement);


    if (sub.getEnableTesting()) {
      controller->weights = (Weights) {
        .k_p = sub.getKp(),
        .k_i = sub.getKi(),
        .k_d = sub.getKd()
      };
    }

    // get change in time
    double dt = 0.1;

    double output = controller->compute_filtered(dt);
    send_thruster_power(output);
  }
}

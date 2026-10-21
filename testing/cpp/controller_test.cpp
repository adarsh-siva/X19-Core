/**
 * @file controller_test.cpp
 * @brief Checks for the depth Controller (PID + EMA filter). Exits non-zero on failure.
 */
#include <cmath>
#include <iostream>

#include "controller.hpp"

static int failures = 0;

static void check(bool ok, const char *what) {
  std::cout << (ok ? "  ok   " : "  FAIL ") << what << "\n";
  if (!ok) failures++;
}

static bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

int main() {
  {
    Controller c({.k_p = 0.5, .k_i = 0, .k_d = 0});
    c.target = 2.0;
    c.record_measurement(1.0);
    check(near(c.compute_filtered(0.1), 0.5), "P term scales with the error (k_p * 1 m)");
  }
  {
    Controller c({.k_p = 0, .k_i = 0, .k_d = 10});
    c.target = 1.0;
    c.record_measurement(0.0);
    check(near(c.compute_filtered(0.1), 0.0), "no derivative kick on the first sample");
  }
  {
    Controller c({.k_p = 1, .k_i = 0, .k_d = 0});
    c.record_measurement(3.0);
    check(near(c.get_measurement(), 3.0), "filter is seeded with the first measurement");
    c.record_measurement(1.0);
    check(near(c.get_measurement(), 2.0), "EMA with alpha 0.5 after seeding");
  }
  {
    Controller c({.k_p = 0, .k_i = 1, .k_d = 0});
    c.target = 10.0;
    c.record_measurement(0.0);
    double out = 0;
    for (int i = 0; i < 100; ++i) out = c.compute_filtered(0.1);
    check(near(out, 1.0), "output clamps at MAX_OUTPUT");
    check(c.integral <= 1.0 + 1e-9, "integral does not wind up while saturated");
  }
  {
    Controller c({.k_p = 1, .k_i = 1, .k_d = 1});
    c.target = 1.0;
    c.record_measurement(0.0);
    c.compute_filtered(0.1);
    c.reset();
    check(c.integral == 0 && c.num_items == 0 && !c.has_measurement, "reset clears PID and filter state");
  }
  std::cout << (failures ? "FAILED\n" : "all passed\n");
  return failures ? 1 : 0;
}

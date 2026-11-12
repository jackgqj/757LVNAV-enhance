#include "bank_controller.h"

#include <cmath>
#include <cstdio>

using namespace lvnv;

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond, msg) do { \
    ++g_checks; \
    if (!(cond)) { \
        ++g_failures; \
        std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    } \
} while (0)

#define CHECK_NEAR(a, b, eps, msg) CHECK(std::abs((a) - (b)) <= (eps), msg)

int main() {
    // Desired track well left of current track: left bank, clamped.
    CHECK_NEAR(ComputeBankCommandDeg(59.0, 117.0, -3.0, 9.0, 1.4, 0.8, 25.0),
               -25.0, 1e-9, "large left track error commands max left bank");
    CHECK_NEAR(ComputeBankCommandDeg(59.0, 85.0, -0.2, 9.0, 1.4, 0.8, 25.0),
               -25.0, 1e-9, "25-30 deg track error holds max bank");

    // On track, no offset: wings level.
    CHECK_NEAR(ComputeBankCommandDeg(59.0, 59.0, 0.0, 10.0, 1.4, 0.8, 25.0),
               0.0, 1e-9, "on track commands zero bank");

    // Right of course: bank left; left of course: bank right.
    CHECK(ComputeBankCommandDeg(59.0, 59.0, 1.0, 10.0, 1.4, 0.8, 25.0) < 0.0,
          "right of course banks left");
    CHECK(ComputeBankCommandDeg(59.0, 59.0, -1.0, 10.0, 1.4, 0.8, 25.0) > 0.0,
          "left of course banks right");

    // Desired right of track: right bank.
    CHECK(ComputeBankCommandDeg(90.0, 60.0, 0.0, 10.0, 1.4, 0.8, 25.0) > 0.0,
          "right track error banks right");

    // Bank limit and gentle small-error behaviour.
    CHECK(std::abs(ComputeBankCommandDeg(0.0, 180.0, 0.0, 10.0, 1.4, 0.8, 20.0)) <= 20.0,
          "bank command respects the configured limit");
    CHECK(std::abs(ComputeBankCommandDeg(59.0, 59.5, 0.01, 20.0, 1.4, 0.8, 25.0)) < 2.0,
          "small error produces a gentle bank");

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

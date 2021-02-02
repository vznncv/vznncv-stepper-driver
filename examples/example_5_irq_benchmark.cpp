/**
 * Helper example to estimate performance of IRQ scheduled steps approach.
 *
 * Usage
 * Press and hold a user button during some seconds to increase target stepper motor position and start movement.
 * During stepper motor movement standard output (UART) will show current stepper motor position.
 * If you press and hold the user button again, target position will be being changed but in the inverse direction.
 */
#include "mbed.h"
#include "vznncv_stepper_motor.h"
#include <chrono>

using namespace std::chrono_literals;
using namespace std::chrono;
using vznncvsteppermotor::BaseStepperMotor;
using vznncvsteppermotor::nanoseconds_u32;

/**
 * Configuration
 */
constexpr nanoseconds_u32 STEPPER_MOTOR_DRIVER_PULSE_WIDTH = 2us;
constexpr PinName APP_USER_LED = LED1;

/**
 * Helper functions
 */
int check_error(int err, const char *expr)
{
    if (err) {
        static char err_buf[128];
        snprintf(err_buf, 128, "Expression \"%s\" has failed with code %i", expr, err);
        MBED_ERROR(MBED_ERROR_INITIALIZATION_FAILED, err_buf);
    }

    return err;
}
#define CHECK_ERROR(expr) check_error(expr, #expr)

/**
 * Main code
 */

static DigitalOut user_led(APP_USER_LED);

class BenchmarkStepperMotor : public BaseStepperMotor {
private:
    nanoseconds_u32 _pulse_width;

public:
    int step_count = 0;

    BenchmarkStepperMotor(nanoseconds_u32 pulse_width)
        : _pulse_width(pulse_width)
    {
    }

protected:
    virtual int step_impl(const step_description_t &step) override
    {
        wait_ns(_pulse_width.count());
        return 0;
    }
    virtual int set_direction_impl(MoveDirection dir) override
    {
        return 0;
    }
};

static BenchmarkStepperMotor stepper_motor(STEPPER_MOTOR_DRIVER_PULSE_WIDTH);

int main()
{
    printf("-- initialize stepper motors --\n");
    CHECK_ERROR(stepper_motor.init());

    printf("-- test stepper motor driver --\n");
    const float max_speed = 1'000'000;
    const int distance = 100'000;
    Timer t;
    t.start();
    CHECK_ERROR(stepper_motor.set_mode_constant_speed(max_speed));
    stepper_motor.move(distance);
    CHECK_ERROR(stepper_motor.wait_end_of_movement());
    t.stop();
    printf("-- finish stepper motor driver testing --\n");

    int max_impl_speed = distance * 1'000'000ll / t.elapsed_time().count();
    printf("Maximal speed (with 100%% CPU usage): %i steps/s\n", max_impl_speed);

    while (true) {
        ThisThread::sleep_for(500ms);
        user_led = !user_led;
    }

    return 0;
}

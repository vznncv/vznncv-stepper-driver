/**
 * Stepper motor usage example arbitrary function.
 *
 * Usage
 * Press and hold a user button during some seconds to increase target stepper motor position and start movement.
 * During stepper motor movement standard output (UART) will show current stepper motor position.
 * If you press and hold the user button again, target position will be being changed but in the inverse direction.
 */
#include "mbed.h"
#include <chrono>
#include <cmath>

#include "vznncv_stepper_motor.h"
#include "vznncv_stepper_motor_extra.h"

using namespace std::chrono_literals;
using namespace std::chrono;

using vznncvsteppermotor::BaseStepperMotor;
using vznncvsteppermotor::microseconds_u32;
using vznncvsteppermotor::SimpleSequenceWrapper;
using vznncvsteppermotor::StepDirDriverStepperMotor;

/**
 * Configuration
 */
constexpr PinName STEPPER_MOTOR_DRIVER_PIN_S = PB_1;
constexpr PinName STEPPER_MOTOR_DRIVER_PIN_D = PB_0;
constexpr PinName STEPPER_MOTOR_DRIVER_PIN_E = PA_7;
constexpr auto STEPPER_MOTOR_DRIVER_FLAGS = StepDirDriverStepperMotor::FLAG_DEFAULT_A4988;
constexpr nanoseconds STEPPER_MOTOR_DRIVER_PULSE_WIDTH = StepDirDriverStepperMotor::PULSE_LENGTH_A4988;
constexpr int STEPPER_MOTOR_SUBSTEP = 4;
constexpr PinName APP_USER_LED = LED1;
constexpr PinName APP_USER_BUTTON = PB_12;

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
static InterruptIn user_button(APP_USER_BUTTON);
static StepDirDriverStepperMotor stepper_motor(STEPPER_MOTOR_DRIVER_PIN_S, STEPPER_MOTOR_DRIVER_PIN_D, STEPPER_MOTOR_DRIVER_PIN_E, STEPPER_MOTOR_DRIVER_FLAGS, STEPPER_MOTOR_DRIVER_PULSE_WIDTH);

class SineSequenceGenerator {
private:
    float _a;
    float _d_phi;
    float _phi;

    static constexpr float _DOUBLE_PI = 6.283185307179586f;
    static constexpr float _DEFAULT_PHASE = _DOUBLE_PI / 4;

public:
    SineSequenceGenerator(float a, float f, microseconds_u32 interval)
    {
        _a = a;
        _d_phi = f * _DOUBLE_PI * interval.count() / 1'000'000;
        _d_phi = fmod(_d_phi, _DOUBLE_PI);
        reset();
    }

    void reset()
    {
        _phi = _DEFAULT_PHASE;
    }

    SimpleSequenceWrapper::sample_t next()
    {
        _phi += _d_phi;
        if (_phi > _DOUBLE_PI) {
            _phi -= _DOUBLE_PI;
        }
        return SimpleSequenceWrapper::sample_t(_a * sinf(_phi));
    }
};

int main()
{
    printf("-- initialize stepper motors --\n");
    CHECK_ERROR(stepper_motor.init());

    // configure sine sequence
    const microseconds_u32 sequence_interval = 10ms;
    const float sine_f = 0.2;
    const float sine_a = 1000 * STEPPER_MOTOR_SUBSTEP;
    SineSequenceGenerator sine_generator(sine_a, sine_f, sequence_interval);
    SimpleSequenceWrapper sine_wrapper(callback(&sine_generator, &SineSequenceGenerator::next), sequence_interval);
    CHECK_ERROR(stepper_motor.set_mode_custom_step(sine_wrapper.get_custom_step_callback()));

    int button_state = 0;
    int prev_button_state = 0;
    Kernel::Clock::time_point prev_movement_update;

    const milliseconds min_message_interval = 200ms;
    Kernel::Clock::time_point prev_message_time = Kernel::Clock::time_point::min();
    Kernel::Clock::time_point current_time;
    int distance_to_go = 0;
    int prev_distance_to_go = -1;

    printf("-- start --\n");
    while (true) {
        current_time = Kernel::Clock::now();
        button_state = user_button.read();
        if (button_state != prev_button_state) {
            if (button_state) {
                stepper_motor.resume_movement();
                user_led = 1;
            } else {
                stepper_motor.pause_movement();
                stepper_motor.set_target_position(0);
                stepper_motor.set_current_position(0);
                sine_generator.reset();
                user_led = 0;
            }
        }
        prev_button_state = button_state;

        // show current distance to go
        distance_to_go = stepper_motor.distance_to_go();
        if (distance_to_go != prev_distance_to_go) {
            if (current_time >= prev_message_time + min_message_interval || distance_to_go == 0) {
                printf("distance_to_go = %i\n", distance_to_go);
                prev_message_time = current_time;
            }
            prev_distance_to_go = distance_to_go;
        }

        // delay
        ThisThread::sleep_for(50ms);
    }

    return 0;
}

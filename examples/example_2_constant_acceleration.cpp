/**
 * Stepper motor usage example with constant acceleration.
 *
 * Usage
 * Press and hold a user button during some seconds to increase target stepper motor position and start movement.
 * During stepper motor movement standard output (UART) will show current stepper motor position.
 * If you press and hold the user button again, target position will be being changed but in the invers direction.
 */
#include "mbed.h"
#include "vznncv_stepper_motor.h"
#include <chrono>

using namespace std::chrono_literals;
using namespace std::chrono;
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
constexpr float STEPPER_MOTOR_MAX_SPEED = 2000.0f;
constexpr float STEPPER_MOTOR_MAX_ACCELERATION = 1000.0f;
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

int main()
{
    printf("-- initialize stepper motors --\n");
    CHECK_ERROR(stepper_motor.init());
    CHECK_ERROR(stepper_motor.set_mode_constant_acceleration(STEPPER_MOTOR_MAX_SPEED * STEPPER_MOTOR_SUBSTEP, STEPPER_MOTOR_MAX_ACCELERATION * STEPPER_MOTOR_SUBSTEP));

    int button_state = 0;
    int prev_button_state = 0;
    int movement_update_dir = -1;
    const float movement_update_k = 2.0f;
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

        if (button_state) {
            if (!prev_button_state) {
                movement_update_dir = -movement_update_dir;
                user_led = !user_led;
            } else {
                int move_step = STEPPER_MOTOR_MAX_SPEED * STEPPER_MOTOR_SUBSTEP * movement_update_k * std::chrono::duration_cast<microseconds>(current_time - prev_movement_update).count() / 1'000'000;
                stepper_motor.move(move_step * movement_update_dir);
            }
            prev_movement_update = current_time;
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

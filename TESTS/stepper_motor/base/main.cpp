#include <cmath>

#include "greentea-client/test_env.h"
#include "mbed.h"
#include "unity.h"
#include "utest.h"

#include "vznncv_stepper_motor.h"
#include "vznncv_stepper_motor_extra.h"

using namespace utest::v1;
using vznncvsteppermotor::BaseStepperMotor;
using vznncvsteppermotor::microseconds_u32;
using vznncvsteppermotor::SimpleSequenceWrapper;

//--------------------------------
// test setup functions
//--------------------------------

static utest::v1::status_t app_test_setup_handler(const size_t number_of_cases)
{
    // common setup code ...
    return greentea_test_setup_handler(number_of_cases);
}

static utest::v1::status_t app_case_setup_handler(const Case *const source, const size_t index_of_case)
{
    // test setup code ...
    return greentea_case_setup_handler(source, index_of_case);
}

static utest::v1::status_t app_case_teardown_handler(const Case *const source, const size_t passed, const size_t failed, const failure_t failure)
{
    // test tear down code ...
    return greentea_case_teardown_handler(source, passed, failed, failure);
}

static void app_test_teardown_handler(const size_t passed, const size_t failed, const failure_t failure)
{
    // common tear down code
    return greentea_test_teardown_handler(passed, failed, failure);
}

//--------------------------------
// dummy stepper motor driver to track steps
//--------------------------------

class DummyStepperMotor : public BaseStepperMotor {
public:
    DummyStepperMotor() = default;

private:
    MoveDirection _last_direction;
    int _total_steps_forward;
    int _total_steps_backward;
    int _enable_event_count;
    int _disable_event_count;
    std::chrono::microseconds _last_step;
    Timer _t;

    int _err_step_impl = 0;
    int _err_set_direction_impl = 0;
    int _err_set_state_impl = 0;

public:
    MoveDirection dsm_get_last_direction() const
    {
        return _last_direction;
    }
    int dsm_get_total_steps_forward() const
    {
        return _total_steps_forward;
    }
    int dsm_get_total_steps_backward() const
    {
        return _total_steps_backward;
    }
    int dsm_get_total_steps() const
    {
        return dsm_get_total_steps_forward() + dsm_get_total_steps_backward();
    }

    float dsm_get_movement_time() const
    {
        return _last_step.count() / 1'000'000.0f;
    }

    int dsm_get_enable_event_count() const
    {
        return _enable_event_count;
    }

    int dsm_get_disable_event_count() const
    {
        return _disable_event_count;
    }

    void dsm_reset_step_timer()
    {
        CriticalSectionLock lock;
        _t.reset();
    }

    void dsm_reset_statistic()
    {
        CriticalSectionLock lock;
        _total_steps_forward = 0;
        _total_steps_backward = 0;
        _enable_event_count = 0;
        _disable_event_count = 0;
        _last_step = 0ms;
        _last_direction = DIR_NONE;
        dsm_reset_step_timer();
    }

    void dsm_err_step_impl(int err)
    {
        _err_step_impl = err;
    }

    void dsm_err_set_direction_impl(int err)
    {
        _err_set_direction_impl = err;
    }

    void dsm_err_set_state_impl(int err)
    {
        _err_set_state_impl = err;
    }

protected:
    // BaseStepperMotor interface
    int init_impl() override
    {
        _t.start();
        dsm_reset_statistic();
        return 0;
    }

    int step_impl(const step_description_t &step) override
    {
        if (_err_step_impl) {
            return _err_step_impl;
        }

        if (_total_steps_forward == 0 && _total_steps_backward == 0) {
            _t.reset();
        }

        switch (_last_direction) {
        case vznncvsteppermotor::BaseStepperMotor::DIR_FORWARD:
            _total_steps_forward++;
            break;
        case vznncvsteppermotor::BaseStepperMotor::DIR_NONE:
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_INVALID_OPERATION), "Step with no direction");
            break;
        case vznncvsteppermotor::BaseStepperMotor::DIR_BACKWARD:
            _total_steps_backward++;
            break;
        default:
            MBED_ERROR(MBED_ERROR_UNKNOWN, "Unreachable code");
        }
        _last_step = _t.elapsed_time();

        return 0;
    }
    int set_direction_impl(MoveDirection dir) override
    {
        if (_err_set_direction_impl) {
            return _err_set_direction_impl;
        }

        _last_direction = dir;
        return 0;
    }

    int set_state_impl(State state) override
    {
        if (_err_set_state_impl) {
            return _err_set_state_impl;
        }

        if (state == STATE_ENABLED) {
            _enable_event_count++;
        } else if (state == STATE_DISABLED) {
            _disable_event_count++;
        } else {
            MBED_ERROR(MBED_ERROR_UNKNOWN, "Unknown state code");
        }
        return 0;
    }
};

static constexpr float PI = 3.141592653589793f;

//--------------------------------
// test functions
//--------------------------------

static void test_simple_forward_movement_with_constant_speed()
{
    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_speed(2000));

    dsm.dsm_reset_statistic();
    dsm.move(100);
    dsm.wait_end_of_movement();

    TEST_ASSERT_EQUAL(100, dsm.dsm_get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.dsm_get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.05f, dsm.dsm_get_movement_time());
}

static void test_simple_backward_movement_with_constant_speed()
{
    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_speed(2000));

    dsm.dsm_reset_statistic();
    dsm.move(-100);
    dsm.wait_end_of_movement();

    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps_forward());
    TEST_ASSERT_EQUAL(100, dsm.dsm_get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.dsm_get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.05f, dsm.dsm_get_movement_time());
}

static void test_simple_forward_movement_with_constant_acceleration()
{
    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_acceleration(2000, 2000));

    dsm.dsm_reset_statistic();
    dsm.move(100);
    dsm.wait_end_of_movement();

    TEST_ASSERT_EQUAL(100, dsm.dsm_get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.dsm_get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.06f, 0.39f, dsm.dsm_get_movement_time());
}

static void test_simple_backward_movement_with_constant_acceleration()
{
    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_acceleration(2000, 2000));

    dsm.dsm_reset_statistic();
    dsm.move(-100);
    dsm.wait_end_of_movement();

    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps_forward());
    TEST_ASSERT_EQUAL(100, dsm.dsm_get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.dsm_get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.06f, 0.39f, dsm.dsm_get_movement_time());
}

struct sine_wave_generator_t {
    sine_wave_generator_t(float a = 0.0f, float phi = 0.0f, float d_phi = 0.1f * PI, int step_count = 0)
        : a(a)
        , phi(phi)
        , d_phi(d_phi)
        , step_count(step_count)
    {
    }

    float a;
    float phi;
    float d_phi;
    int step_count;

    SimpleSequenceWrapper::sample_t next()
    {
        SimpleSequenceWrapper::sample_t sample;

        if (step_count <= 0) {
            sample.value = 0;
            sample.flags = SimpleSequenceWrapper::FLAG_STOP;
        } else {
            sample.value = a * sinf(phi);
            phi += d_phi;
            sample.flags = 0;
            step_count--;
        }

        return sample;
    }
};

static void test_simple_movement_with_custom_step()
{
    const microseconds_u32 interval = 10ms;
    const float f = 2.0f;
    const int num_periods = 3;

    sine_wave_generator_t sine_wave_generator;
    sine_wave_generator.a = 500.0f;
    sine_wave_generator.d_phi = 2 * PI * f * interval.count() / 1'000'000.0f;
    sine_wave_generator.step_count = num_periods * 1'000'000.0f / (f * interval.count());

    SimpleSequenceWrapper seq_wrapper(callback(&sine_wave_generator, &sine_wave_generator_t::next), interval);
    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());

    TEST_ASSERT_EQUAL(0, dsm.set_mode_custom_step(seq_wrapper.get_custom_step_callback()));
    dsm.dsm_reset_statistic();

    TEST_ASSERT_EQUAL(0, dsm.resume_movement());
    TEST_ASSERT_EQUAL(0, dsm.wait_end_of_movement());

    const int expected_one_dir_steps = sine_wave_generator.a * 2 * num_periods;
    const float expected_move_time = num_periods / f;
    TEST_ASSERT_EQUAL(0, sine_wave_generator.step_count);
    TEST_ASSERT_INT_WITHIN(100, expected_one_dir_steps, dsm.dsm_get_total_steps_forward());
    TEST_ASSERT_INT_WITHIN(100, expected_one_dir_steps, dsm.dsm_get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.dsm_get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.2f, expected_move_time, dsm.dsm_get_movement_time());
}

static void test_complex_movement()
{
    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());

    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_acceleration(2000, 1000));
    dsm.dsm_reset_statistic();
    dsm.move(100);
    ThisThread::sleep_for(500ms);
    dsm.move_to(-500);
    ThisThread::sleep_for(500ms);
    dsm.move(-1000);
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_acceleration(4000, 4000));
    ThisThread::sleep_for(500ms);
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_speed(4000));
    dsm.move(2000);
    dsm.wait_end_of_movement();

    TEST_ASSERT_INT_WITHIN(50, 1160, dsm.dsm_get_total_steps_forward());
    TEST_ASSERT_INT_WITHIN(50, 660, dsm.dsm_get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.dsm_get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 1.8f, dsm.dsm_get_movement_time());
}

void test_base_enable_disable_functionality()
{
    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_speed(1000));

    // check default state
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_enable_event_count());
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_disable_event_count());

    // check disabling
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_DISABLED));
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_DISABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_disable_event_count());
    // check idempotence
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_DISABLED));
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_DISABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_disable_event_count());
    // check enabling
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_ENABLED));
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(2, dsm.dsm_get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_disable_event_count());
    // check idempotence
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_ENABLED));
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(2, dsm.dsm_get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_disable_event_count());
}

void test_active_base_enable_disable_functionality()
{
    int steps;
    int dist_to_go;
    Timer t;
    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_speed(1000));
    dsm.dsm_reset_statistic();

    // move, but stop at the center
    dsm.move(200);
    ThisThread::sleep_for(100ms);
    dsm.set_state(BaseStepperMotor::STATE_DISABLED);
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_DISABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_disable_event_count());
    dist_to_go = dsm.distance_to_go();
    steps = dsm.dsm_get_total_steps_forward();
    TEST_ASSERT_INT_WITHIN(25, 100, steps);
    TEST_ASSERT_INT_WITHIN(25, 100, dist_to_go);
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps_backward());

    // wait and check that there is no more movement
    ThisThread::sleep_for(200ms);
    TEST_ASSERT_EQUAL(steps, dsm.dsm_get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps_backward());
    TEST_ASSERT_EQUAL(dist_to_go, dsm.distance_to_go());

    // check that resume method doesn't work
    TEST_ASSERT_EQUAL(0, dsm.resume_movement());
    ThisThread::sleep_for(200ms);
    TEST_ASSERT_EQUAL(steps, dsm.dsm_get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps_backward());
    TEST_ASSERT_EQUAL(dist_to_go, dsm.distance_to_go());

    // check that ::wait_stopping resum control immediately
    t.start();
    TEST_ASSERT_EQUAL(0, dsm.wait_end_of_movement());
    t.stop();
    // note: time should be small, but any interrupt may increase it (like OS scheduler), so check it with some reserve
    TEST_ASSERT_INT_WITHIN(100, 0, t.elapsed_time().count());

    // resume movement
    dsm.dsm_reset_step_timer();
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_ENABLED));
    TEST_ASSERT_EQUAL(0, dsm.resume_movement());
    TEST_ASSERT_EQUAL(0, dsm.wait_end_of_movement());
    // check results
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.dsm_get_disable_event_count());
    TEST_ASSERT_EQUAL(200, dsm.dsm_get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.dsm_get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.1f, dsm.dsm_get_movement_time());
}

static void test_configuration_methods()
{
    float max_speed;
    float max_accelration;
    BaseStepperMotor::CustomStepCallback custom_step_cb;

    BaseStepperMotor::CustomStepCallback custom_step_cb_impl = [](const BaseStepperMotor::position_t &pos) -> BaseStepperMotor::step_instruction_t {
        return { BaseStepperMotor::DIR_NONE, 500us };
    };

    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());

    // check constant speed parameters
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_speed(1200.0f));
    max_speed = 0.0f;
    TEST_ASSERT_EQUAL(0, dsm.get_mode_params_constant_speed(max_speed));
    TEST_ASSERT_EQUAL(1200.0f, max_speed);
    TEST_ASSERT_EQUAL(BaseStepperMotor::MODE_CONSTANT_SPEED, dsm.get_mode());
    TEST_ASSERT_NOT_EQUAL(0, dsm.get_mode_params_constant_acceleration(max_speed, max_accelration));
    TEST_ASSERT_NOT_EQUAL(0, dsm.get_mode_params_custom_step(custom_step_cb));
    // check constant acceleration parameters
    TEST_ASSERT_EQUAL(0, dsm.set_mode_constant_acceleration(800.0f, 500.0f));
    max_speed = 0.0f;
    max_accelration = 0.0f;
    TEST_ASSERT_EQUAL(0, dsm.get_mode_params_constant_acceleration(max_speed, max_accelration));
    TEST_ASSERT_EQUAL(800.0f, max_speed);
    TEST_ASSERT_EQUAL(500.0f, max_accelration);
    TEST_ASSERT_EQUAL(BaseStepperMotor::MODE_CONSTANT_ACCELERATION, dsm.get_mode());
    TEST_ASSERT_NOT_EQUAL(0, dsm.get_mode_params_constant_speed(max_speed));
    TEST_ASSERT_NOT_EQUAL(0, dsm.get_mode_params_custom_step(custom_step_cb));
    // check custom step
    TEST_ASSERT_EQUAL(0, dsm.set_mode_custom_step(custom_step_cb_impl));
    custom_step_cb = 0;
    TEST_ASSERT_EQUAL(0, dsm.get_mode_params_custom_step(custom_step_cb));
    TEST_ASSERT_TRUE(custom_step_cb == custom_step_cb_impl);
    TEST_ASSERT_EQUAL(BaseStepperMotor::MODE_CUSTOM_STEP, dsm.get_mode());
    TEST_ASSERT_NOT_EQUAL(0, dsm.get_mode_params_constant_speed(max_speed));
    TEST_ASSERT_NOT_EQUAL(0, dsm.get_mode_params_constant_acceleration(max_speed, max_accelration));

    // check step parameters
    dsm.set_mode_constant_speed(1000.0f);
    TEST_ASSERT_EQUAL(0, dsm.get_target_position());
    TEST_ASSERT_EQUAL(0, dsm.get_current_position());
    TEST_ASSERT_EQUAL(0, dsm.distance_to_go());
    dsm.set_target_position(100);
    dsm.set_current_position(50);
    TEST_ASSERT_EQUAL(100, dsm.get_target_position());
    TEST_ASSERT_EQUAL(50, dsm.get_current_position());
    TEST_ASSERT_EQUAL(50, dsm.distance_to_go());
    dsm.set_target_position(-20);
    dsm.set_current_position(80);
    TEST_ASSERT_EQUAL(-20, dsm.get_target_position());
    TEST_ASSERT_EQUAL(80, dsm.get_current_position());
    TEST_ASSERT_EQUAL(-100, dsm.distance_to_go());
}

void test_impl_errors()
{
    DummyStepperMotor dsm;
    TEST_ASSERT_EQUAL(0, dsm.init());
    dsm.set_mode_constant_speed(1000.0f);
    TEST_ASSERT_EQUAL(0, dsm.get_error());
    auto reset_postions = [](BaseStepperMotor &bsm) {
        bsm.set_current_position(0);
        bsm.set_target_position(0);
    };

    // check that we movement doesn't cause errors
    dsm.move(10);
    dsm.wait_end_of_movement();
    TEST_ASSERT_EQUAL(0, dsm.get_error());

    // check that step error prevents movement
    dsm.dsm_reset_statistic();
    reset_postions(dsm);
    dsm.dsm_err_step_impl(-1);
    dsm.move(10);
    TEST_ASSERT_NOT_EQUAL(0, dsm.wait_end_of_movement());
    TEST_ASSERT_NOT_EQUAL(0, dsm.get_error());
    TEST_ASSERT_EQUAL(10, dsm.distance_to_go());
    TEST_ASSERT_EQUAL(0, dsm.get_current_position());
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps());
    dsm.dsm_err_step_impl(0);
    dsm.clear_error();
    TEST_ASSERT_EQUAL(0, dsm.resume_movement());
    TEST_ASSERT_EQUAL(0, dsm.wait_end_of_movement());
    TEST_ASSERT_EQUAL(0, dsm.get_error());
    TEST_ASSERT_EQUAL(0, dsm.distance_to_go());
    TEST_ASSERT_EQUAL(10, dsm.get_current_position());
    TEST_ASSERT_EQUAL(10, dsm.dsm_get_total_steps());

    // check that direction error prevents movement
    dsm.dsm_reset_statistic();
    reset_postions(dsm);
    dsm.dsm_err_set_direction_impl(-1);
    dsm.move(5);
    TEST_ASSERT_NOT_EQUAL(0, dsm.wait_end_of_movement());
    TEST_ASSERT_NOT_EQUAL(0, dsm.get_error());
    TEST_ASSERT_EQUAL(5, dsm.distance_to_go());
    TEST_ASSERT_EQUAL(0, dsm.get_current_position());
    TEST_ASSERT_EQUAL(0, dsm.dsm_get_total_steps());
    dsm.dsm_err_set_direction_impl(0);
    dsm.clear_error();
    TEST_ASSERT_EQUAL(0, dsm.resume_movement());
    TEST_ASSERT_EQUAL(0, dsm.wait_end_of_movement());
    TEST_ASSERT_EQUAL(0, dsm.get_error());
    TEST_ASSERT_EQUAL(0, dsm.distance_to_go());
    TEST_ASSERT_EQUAL(5, dsm.get_current_position());
    TEST_ASSERT_EQUAL(5, dsm.dsm_get_total_steps());

    // check set state error
    dsm.dsm_reset_statistic();
    reset_postions(dsm);
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    dsm.dsm_err_set_state_impl(-1);
    TEST_ASSERT_NOT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_DISABLED));
    TEST_ASSERT_NOT_EQUAL(0, dsm.get_error());
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    dsm.dsm_err_set_state_impl(0);
    dsm.clear_error();
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_DISABLED));
    TEST_ASSERT_EQUAL(0, dsm.get_error());
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_DISABLED, dsm.get_state());
}

// test cases description
#define SimpleCase(test_fun) Case(#test_fun, app_case_setup_handler, test_fun, app_case_teardown_handler, greentea_case_failure_continue_handler)
static Case cases[] = {
    // base modes
    SimpleCase(test_simple_forward_movement_with_constant_acceleration),
    SimpleCase(test_simple_backward_movement_with_constant_acceleration),
    SimpleCase(test_simple_forward_movement_with_constant_speed),
    SimpleCase(test_simple_backward_movement_with_constant_speed),
    SimpleCase(test_simple_movement_with_custom_step),
    // combination of modes
    SimpleCase(test_complex_movement),
    SimpleCase(test_base_enable_disable_functionality),
    SimpleCase(test_active_base_enable_disable_functionality),
    // test configuration methods
    SimpleCase(test_configuration_methods),
    SimpleCase(test_impl_errors)
};
static Specification specification(app_test_setup_handler, cases, app_test_teardown_handler);

// Entry point into the tests
int main()
{
    // host handshake
    // note: should be invoked here or in the test_setup_handler
    GREENTEA_SETUP(40, "default_auto");
    // run tests
    return !Harness::run(specification);
}

#include "greentea-client/test_env.h"
#include "mbed.h"
#include "unity.h"
#include "utest.h"

#include "vznncv_stepper_motor.h"
using namespace utest::v1;
using vznncvsteppermotor::BaseStepperMotor;

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
    DummyStepperMotor(float max_speed = DEFAULT_MAX_SPEED, float max_acceleration = DEFAULT_MAX_ACCELERATION)
        : BaseStepperMotor(max_speed, max_acceleration)
    {
    }

private:
    MoveDirection _last_direction;
    int _total_steps_forward;
    int _total_steps_backward;
    int _enable_event_count;
    int _disable_event_count;
    std::chrono::microseconds _last_step;
    Timer _t;

public:
    MoveDirection get_last_direction() const
    {
        return _last_direction;
    }
    int get_total_steps_forward() const
    {
        return _total_steps_forward;
    }
    int get_total_steps_backward() const
    {
        return _total_steps_backward;
    }

    float get_move_time() const
    {
        return _last_step.count() / 1'000'000.0f;
    }

    int get_enable_event_count() const
    {
        return _enable_event_count;
    }

    int get_disable_event_count() const
    {
        return _disable_event_count;
    }

    void reset_step_timer()
    {
        CriticalSectionLock lock;
        _t.reset();
    }

    void reset_statistic()
    {
        CriticalSectionLock lock;
        _total_steps_forward = 0;
        _total_steps_backward = 0;
        _enable_event_count = 0;
        _disable_event_count = 0;
        _last_step = 0ms;
        _last_direction = DIR_NONE;
        reset_step_timer();
    }

protected:
    // BaseStepperMotor interface
    int init_impl() override
    {
        _t.start();
        reset_statistic();
        return 0;
    }

    int step_impl(int32_t step) override
    {
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
        _last_direction = dir;
        return 0;
    }

    int set_state_impl(State state) override
    {
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

//--------------------------------
// test functions
//--------------------------------

static void test_simple_forward_moving_with_acceleration()
{
    DummyStepperMotor dsm(2000, 2000);
    TEST_ASSERT_EQUAL(0, dsm.init());

    dsm.reset_statistic();
    dsm.move(100);
    dsm.wait_stopping();

    TEST_ASSERT_EQUAL(100, dsm.get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.06f, 0.39f, dsm.get_move_time());
}

static void test_simple_backward_moving_with_acceleration()
{
    DummyStepperMotor dsm(2000, 2000);
    TEST_ASSERT_EQUAL(0, dsm.init());

    dsm.reset_statistic();
    dsm.move(-100);
    dsm.wait_stopping();

    TEST_ASSERT_EQUAL(0, dsm.get_total_steps_forward());
    TEST_ASSERT_EQUAL(100, dsm.get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.06f, 0.39f, dsm.get_move_time());
}

static void test_simple_forward_moving_with_constant_speed()
{
    DummyStepperMotor dsm(2000, -1);
    TEST_ASSERT_EQUAL(0, dsm.init());

    dsm.reset_statistic();
    dsm.move(100);
    dsm.wait_stopping();

    TEST_ASSERT_EQUAL(100, dsm.get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.05f, dsm.get_move_time());
}

static void test_simple_backward_moving_with_constant_speed()
{
    DummyStepperMotor dsm(2000, -1);
    TEST_ASSERT_EQUAL(0, dsm.init());

    dsm.reset_statistic();
    dsm.move(-100);
    dsm.wait_stopping();

    TEST_ASSERT_EQUAL(0, dsm.get_total_steps_forward());
    TEST_ASSERT_EQUAL(100, dsm.get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.05f, dsm.get_move_time());
}

static void test_complex_movement()
{
    DummyStepperMotor dsm(2000, 1000);
    TEST_ASSERT_EQUAL(0, dsm.init());

    dsm.reset_statistic();
    dsm.move(100);
    ThisThread::sleep_for(500ms);
    dsm.move_to(-500);
    ThisThread::sleep_for(500ms);
    dsm.move(-1000);
    dsm.set_max_speed(4000);
    dsm.set_max_acceleration(4000);
    ThisThread::sleep_for(500ms);
    dsm.set_max_acceleration(-1);
    dsm.move(2000);
    dsm.wait_stopping();

    TEST_ASSERT_INT_WITHIN(50, 1360, dsm.get_total_steps_forward());
    TEST_ASSERT_INT_WITHIN(50, 860, dsm.get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 1.8f, dsm.get_move_time());
}

void test_base_enable_disable_functionality()
{
    DummyStepperMotor dsm(1000, -1);
    TEST_ASSERT_EQUAL(0, dsm.init());

    // check default state
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(1, dsm.get_enable_event_count());
    TEST_ASSERT_EQUAL(0, dsm.get_disable_event_count());

    // check disabling
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_DISABLED));
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_DISABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(1, dsm.get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.get_disable_event_count());
    // check idempotence
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_DISABLED));
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_DISABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(1, dsm.get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.get_disable_event_count());
    // check enabling
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_ENABLED));
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(2, dsm.get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.get_disable_event_count());
    // check idempotence
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_ENABLED));
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(2, dsm.get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.get_disable_event_count());
}

void test_active_base_enable_disable_functionality()
{
    int steps;
    int dist_to_go;
    Timer t;
    DummyStepperMotor dsm(1000, -1);
    TEST_ASSERT_EQUAL(0, dsm.init());
    dsm.reset_statistic();

    // move, but stop at the center
    dsm.move(200);
    ThisThread::sleep_for(100ms);
    dsm.set_state(BaseStepperMotor::STATE_DISABLED);
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_DISABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(0, dsm.get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.get_disable_event_count());
    dist_to_go = dsm.distance_to_go();
    steps = dsm.get_total_steps_forward();
    TEST_ASSERT_INT_WITHIN(25, 100, steps);
    TEST_ASSERT_INT_WITHIN(25, 100, dist_to_go);
    TEST_ASSERT_EQUAL(0, dsm.get_total_steps_backward());

    // wait and check that there is no more movement
    ThisThread::sleep_for(200ms);
    TEST_ASSERT_EQUAL(steps, dsm.get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.get_total_steps_backward());
    TEST_ASSERT_EQUAL(dist_to_go, dsm.distance_to_go());

    // check that resume method doesn't work
    TEST_ASSERT_EQUAL(0, dsm.resume_movement());
    ThisThread::sleep_for(200ms);
    TEST_ASSERT_EQUAL(steps, dsm.get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.get_total_steps_backward());
    TEST_ASSERT_EQUAL(dist_to_go, dsm.distance_to_go());

    // check that ::wait_stopping resum control immediately
    t.start();
    TEST_ASSERT_EQUAL(0, dsm.wait_stopping());
    t.stop();
    // note: time should be small, but any interrupt may increase it (like OS scheduler), so check it with some reserve
    TEST_ASSERT_INT_WITHIN(100, 0, t.elapsed_time().count());

    // resume movement
    dsm.reset_step_timer();
    TEST_ASSERT_EQUAL(0, dsm.set_state(BaseStepperMotor::STATE_ENABLED));
    TEST_ASSERT_EQUAL(0, dsm.resume_movement());
    TEST_ASSERT_EQUAL(0, dsm.wait_stopping());
    // check results
    TEST_ASSERT_EQUAL(BaseStepperMotor::STATE_ENABLED, dsm.get_state());
    TEST_ASSERT_EQUAL(1, dsm.get_enable_event_count());
    TEST_ASSERT_EQUAL(1, dsm.get_disable_event_count());
    TEST_ASSERT_EQUAL(200, dsm.get_total_steps_forward());
    TEST_ASSERT_EQUAL(0, dsm.get_total_steps_backward());
    TEST_ASSERT_EQUAL(BaseStepperMotor::DIR_NONE, dsm.get_last_direction());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.1f, dsm.get_move_time());
}

// test cases description
#define SimpleCase(test_fun) Case(#test_fun, app_case_setup_handler, test_fun, app_case_teardown_handler, greentea_case_failure_continue_handler)
static Case cases[] = {
    SimpleCase(test_simple_forward_moving_with_acceleration),
    SimpleCase(test_simple_backward_moving_with_acceleration),
    SimpleCase(test_simple_forward_moving_with_constant_speed),
    SimpleCase(test_simple_backward_moving_with_constant_speed),
    SimpleCase(test_complex_movement),
    SimpleCase(test_base_enable_disable_functionality),
    SimpleCase(test_active_base_enable_disable_functionality)
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

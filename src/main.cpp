#include "main.h"
#include "Eigen/Dense"
#include "sysid/oneDofVelocitySystem.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup left_mg({-8, -9, 10});
    pros::MotorGroup right_mg({3, 4, -2});

    left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    right_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);

    double maxVelocity = 1.0;

    OneDofVelocitySystem sys;

    sys.characterize([&left_mg, &right_mg]() {
                         return (left_mg.get_actual_velocity() - right_mg.get_actual_velocity()) / 200.0;
                     }, [master, &left_mg, &right_mg]() mutable {
                         double u = master.get_analog(ANALOG_RIGHT_X) / 127.0;
                         left_mg.move_voltage(12000.0 * u);
                         right_mg.move_voltage(-12000.0 * u);
                         return u;
                     }, [&master]() { return master.get_digital(DIGITAL_A); });

    double lastInput = 0.0;
    double acceleration = 0.0;

    auto ff = sys.getFF();

    pros::lcd::print(0, "Kv, Ka, Ks: %f, %f, %f", ff(0, 0), ff(0, 1), ff(0, 2));

    while (true) {
        const double velocity = master.get_analog(ANALOG_RIGHT_X) * maxVelocity / 127.0;
        // Gets the turn left/right from right joystick

        acceleration = (velocity - lastInput) / 0.01;

        double voltage = sys.evaluate(Eigen::Vector3d(velocity, acceleration, signnum(velocity))) * 12000;
        left_mg.move_voltage(voltage);
        right_mg.move_voltage(-voltage);

        lastInput = velocity;

        pros::delay(10); // Run for 20 ms then update
    }
}

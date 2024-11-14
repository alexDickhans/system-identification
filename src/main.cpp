#include "main.h"
#include "Eigen/Dense"

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

double signnum(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
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
    pros::MotorGroup left_mg({-12, 13});

    left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);

    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> u;

    double lastVelocity = 0.0;
    double maxVelocity = 0.0;

    while (!master.get_digital(DIGITAL_A)) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0); // Prints status of the emulated screen LCDs

        // Arcade control scheme
        int turn = master.get_analog(ANALOG_RIGHT_X); // Gets the turn left/right from right joystick

        left_mg.move(turn);

        u.emplace_back(turn);
        velocity.emplace_back(left_mg.get_actual_velocity());
        acceleration.emplace_back((left_mg.get_actual_velocity() - lastVelocity) / 0.01);

        if (abs(left_mg.get_actual_velocity()) > maxVelocity) {
            maxVelocity = abs(left_mg.get_actual_velocity());
        }

        pros::delay(10); // Run for 20 ms then update
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(velocity.size() - 1, 3);
    Eigen::MatrixXd b = Eigen::VectorXd::Zero(velocity.size() - 1);

    for (int i = 0; i < velocity.size() - 1; i++) {
        A(i, 0) = velocity[i];
        A(i, 1) = u[i];
        A(i, 2) = -signnum(velocity[i]);
        b(i) = velocity[i + 1];
    }

    Eigen::VectorXd solution = A.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

    std::cout << "The least-squares solution is:\n"
            << solution << std::endl;

    double K_s = -solution(2) / solution(1);
    double K_v = (1 - solution(0)) / solution(1);
    double K_a = -K_v * 0.01 / log(solution(0));

    std::cout << "K_s: " << K_s << std::endl;
    std::cout << "K_v: " << K_v << std::endl;
    std::cout << "K_a: " << K_a << std::endl;

    Eigen::Vector3d ff = {K_v, K_a, K_s};

    double lastInput = 0.0;

    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0); // Prints status of the emulated screen LCDs

        // Arcade control scheme
        int turn = master.get_analog(ANALOG_RIGHT_X) * maxVelocity / 127.0; // Gets the turn left/right from right joystick

        double acceleration = (turn - lastInput) / 0.01;

        left_mg.move(Eigen::RowVector3d(turn, acceleration, signnum(turn)) * ff);

        lastInput = turn;

        pros::delay(10); // Run for 20 ms then update
    }
}

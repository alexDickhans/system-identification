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

std::pair<double, double> desaturate(double x, double y) {
    double max = std::max(x, y);

    if (abs(max) > 1.0) {
        return std::make_pair(x / max, y / max);
    }
    return std::make_pair(x, y);
}

std::pair<double, double> convert(std::pair<double, double> p) {
    return std::make_pair((p.first + p.second) / 2.0, (p.first - p.second) / 2.0);
}

static std::string toString(const Eigen::MatrixXd &mat) {
    std::stringstream ss;
    ss << mat;
    return ss.str();
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

    std::vector<std::pair<double, double> > velocity;
    std::vector<std::pair<double, double> > u;

    double maxLinearVelocity = 0.0, maxAngularVelocity = 0.0;

    while (!master.get_digital(DIGITAL_A)) {
        // Arcade control scheme
        double linear = master.get_analog(ANALOG_LEFT_Y) / 127.0; // Gets the turn left/right from right joystick
        double turn = master.get_analog(ANALOG_RIGHT_X) / 127.0; // Gets the turn left/right from right joystick

        auto desaturated = desaturate(linear + turn, linear - turn);

        left_mg.move_voltage(desaturated.first * 12000);
        right_mg.move_voltage(desaturated.second * 12000);

        auto converted_input = convert(desaturated);
        auto converted_output = convert(std::make_pair(left_mg.get_actual_velocity(), right_mg.get_actual_velocity()));

        u.emplace_back(converted_input.first, converted_input.second);
        velocity.emplace_back(converted_output.first, converted_output.second);

        if (abs(converted_output.first) > maxLinearVelocity) {
            maxLinearVelocity = abs(converted_output.first);
        }

        if (abs(converted_output.second) > maxAngularVelocity) {
            maxAngularVelocity = abs(converted_output.second);
        }

        pros::delay(10); // Run for 20 ms then update
    }

    Eigen::MatrixXd ALinear = Eigen::MatrixXd::Zero(velocity.size() - 1, 2);
    Eigen::MatrixXd bLinear = Eigen::VectorXd::Zero(velocity.size() - 1);

    for (int i = 0; i < velocity.size() - 1; i++) {
        ALinear(i, 0) = velocity[i].first;
        ALinear(i, 1) = u[i].first;
        bLinear(i) = velocity[i + 1].first;
    }

    Eigen::MatrixXd AAngular = Eigen::MatrixXd::Zero(velocity.size() - 1, 2);
    Eigen::MatrixXd bAngular = Eigen::VectorXd::Zero(velocity.size() - 1);

    for (int i = 0; i < velocity.size() - 1; i++) {
        AAngular(i, 0) = velocity[i].second;
        AAngular(i, 1) = u[i].second;
        bAngular(i) = velocity[i + 1].second;
    }

    std::cout << ALinear << std::endl;
    std::cout << bLinear << std::endl;
    std::cout << AAngular << std::endl;
    std::cout << bAngular << std::endl;

    Eigen::VectorXd solutionLinear = ALinear.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().
            solve(bLinear);
    Eigen::VectorXd solutionAngular = AAngular.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().
            solve(bAngular);

    std::pair<double, double> last{0.0, 0.0};

    Eigen::Vector2d ffLinear = {
        (1 - solutionLinear(0)) / solutionLinear(1),
        (solutionLinear(0) - 1.0) * 0.01 / (log(solutionLinear(0) * solutionLinear(1)))
    };
    Eigen::Vector2d ffAngular = {
        (1 - solutionAngular(0)) / solutionAngular(1),
        (solutionAngular(0) - 1.0) * 0.01 / (log(solutionAngular(0) * solutionAngular(1)))
    };

    // Make a matrix with the linear and angular solutions
    Eigen::Matrix2d solution;
    solution.col(0) = ffLinear;
    solution.col(1) = ffAngular;

    pros::lcd::print(0, "%s", toString(solution)); // Prints status of the emulated screen LCDs

    while (true) {

        // Arcade control scheme
        double linear = master.get_analog(ANALOG_LEFT_Y) * maxLinearVelocity / 127.0;
        // Gets the turn left/right from right joystick
        double turn = master.get_analog(ANALOG_RIGHT_X) * maxAngularVelocity / 127.0;
        // Gets the turn left/right from right joystick

        auto desaturated = convert(desaturate(linear + turn, linear - turn));

        Eigen::RowVector2d linearRow = {desaturated.first, (desaturated.first - last.first) / 0.01};
        Eigen::RowVector2d angularRow = {desaturated.second, (desaturated.second - last.second) / 0.01};

        double linearResult = linearRow * ffLinear;
        double angularResult = angularRow * ffAngular;

        left_mg.move((linearResult + angularResult) * 12000);
        right_mg.move((linearResult - angularResult) * 12000);

        pros::delay(10); // Run for 20 ms then update
    }
}

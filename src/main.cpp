#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-3, -4, -5},     // Left Chassis Ports (negative port will reverse it!)
    {6, 7, 8},  // Right Chassis Ports (negative port will reverse it!)

    11,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    360);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel horiz_tracker(11, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels
pros::Distance distance_sensor(12);





















//THE CASCADE LIFT  IS THE LIFT BAR THAT IS ON THE CASCADE

const int numStates = 7;
int statesLow[numStates]  = {190, 0, 160, 460, 1300, 1800, 2300};   // alliance (low) goal — already tuned
int statesHigh[numStates] = {190, 0, 350, 800, 1300, 1800, 2300};   // high goal — placeholder, tune later
int currState = 0;
int target = 0;

bool highGoalMode = false;  // toggled by L2
bool pistonClamped = false;  // toggled by L1

//For the controler screen
bool last_clamped_state = false;
bool last_high_goal_state = false;
int controller_print_counter = 0;



int chainStatesLow[numStates]  = {100, 1470, 1250, 1165, 1400, 1400, 1400};
int chainStatesHigh[numStates] = {100, 1200, 1400, 1400, 1400, 1400, 1400};  // placeholder, tune later
int chainTarget = 0;
const int CHAIN_TOLERANCE = 20;  // degrees, how close counts as "arrived" — tune later




// --- new stuff for the "hop before dropping" behavior ---
enum LiftPhase { LIFT_IDLE, LIFT_HOPPING, LIFT_MOVING };
LiftPhase liftPhase = LIFT_MOVING;
int pendingTarget = 0;
const int LIFT_HOP_DEGREES = 300;   // how far to pop up before descending
extern const int LIFT_TOLERANCE = 100;     // how close (in motor degrees) counts as "reached" the hop point






bool liftResetting = false;   // true while resetLiftAndChain() is homing the lift
bool chainResetting = false;  // true while resetLiftAndChain() is homing the chain bar
  

void liftControl() {
  if (liftResetting) return;  // let resetLiftAndChain() drive the motor directly

  // Hop phase logic remains exactly the same
  if (liftPhase == LIFT_HOPPING) {
    double hopError = target - liftbar_motors.get_position();
    if (fabs(hopError) < LIFT_TOLERANCE) {
      target = pendingTarget;
      liftPhase = LIFT_MOVING;
    }
  }

  // Use PROS built-in absolute positioning instead of custom proportional control
  // Note: Adjust the '200' to match your motor's gear cartridge max RPM 
  // (100 for Red, 200 for Green, 600 for Blue)
  liftbar_motors.move_absolute(target, 600); 
}




void liftSetState(int newTarget) {
  double currentPos = liftbar_motors.get_position();

  if (newTarget < currentPos) {
    // Moving down: hop up 90 degrees first, then descend
    pendingTarget = newTarget;
    target = currentPos + LIFT_HOP_DEGREES;
    liftPhase = LIFT_HOPPING;
  } else {
    // Moving up (or staying): go straight there
    target = newTarget;
    liftPhase = LIFT_MOVING;
  }
}



// Scoring macro state machine
enum ScorePhase { SCORE_IDLE, SCORE_RAISING, SCORE_HOPPING, SCORE_WAIT_UNCLAMP, SCORE_CHAIN_RETURN, SCORE_DESCENDING };
ScorePhase scorePhase = SCORE_IDLE;
int chainRestAngle = 0;  // captured fresh each time the macro starts

void chainControl() {
  double kp = 0.5;
  if (chainResetting) return;  // let resetLiftAndChain() drive the motor directly
  double error = chainTarget - chainbar_motors.get_position();
  double velocity = kp * error;
  chainbar_motors.move(velocity);
}

const int LIFT_RESET_VOLTAGE = 3000;    // flipped for lift
const int CHAIN_RESET_VOLTAGE = -3000;  // unchanged for chain
const int RESET_DRIVE_TIME = 300;       // ms — just drive into the stop for this long, then zero

const int LIFT_RESET_CURRENT_LIMIT = 125;   // mA — tune this down until it stops gently
const int LIFT_NORMAL_CURRENT_LIMIT = 2500; // V5 smart motor default max

void resetLiftAndChain() {
  liftResetting = true;
  chainResetting = true;

  liftbar_motors.set_current_limit(LIFT_RESET_CURRENT_LIMIT);  // lower torque just for homing

  liftbar_motors.move_voltage(LIFT_RESET_VOLTAGE);
  chainbar_motors.move_voltage(CHAIN_RESET_VOLTAGE);

  pros::delay(RESET_DRIVE_TIME);

  liftbar_motors.move_voltage(0);
  liftbar_motors.tare_position();
  liftbar_motors.set_current_limit(LIFT_NORMAL_CURRENT_LIMIT);  // restore full torque for normal operation

  chainbar_motors.move_voltage(0);
  chainbar_motors.tare_position();

  target = 0;
  pendingTarget = 0;
  liftPhase = LIFT_MOVING;
  liftResetting = false;

  chainTarget = 0;
  chainResetting = false;
}






void startScoreSequence(int stateIndex) {
  chainRestAngle = chainbar_motors.get_position();  // remember where the chainbar started

  int* activeStates      = highGoalMode ? statesHigh      : statesLow;
  int* activeChainStates = highGoalMode ? chainStatesHigh : chainStatesLow;

  target = activeStates[stateIndex];
  chainTarget = activeChainStates[stateIndex];
  liftPhase = LIFT_MOVING;          // going straight to the target
  
  if (stateIndex == 0) {
    // If it's state 0, bypass the scoring macro entirely.
    // The motors will just move directly to the target without hopping.
    scorePhase = SCORE_IDLE;
  } else {
    // For all other states, start the normal scoring sequence.
    scorePhase = SCORE_RAISING;
  }
}

void scoreMacroUpdate() {
  switch (scorePhase) {
    case SCORE_RAISING: {
      bool liftReady = fabs(target - liftbar_motors.get_position()) < LIFT_TOLERANCE;
      bool chainReady = fabs(chainTarget - chainbar_motors.get_position()) < CHAIN_TOLERANCE;
      if (liftReady && chainReady) {
        // at scoring height — wait for the driver to release the clamp
        scorePhase = SCORE_WAIT_UNCLAMP;
      }
      break;
    }

    case SCORE_WAIT_UNCLAMP: {
     if (piston.get()) {   // true = unclamped -> this is what continues the sequence
      pros::delay(200);
      pistonClamped = false;

      target = liftbar_motors.get_position() + LIFT_HOP_DEGREES;
      liftPhase = LIFT_MOVING;
      scorePhase = SCORE_HOPPING;
      }
      break;
    }

    case SCORE_HOPPING: {
      if (fabs(target - liftbar_motors.get_position()) < LIFT_TOLERANCE) {
        chainTarget = 0;   // always go home, not wherever it started
        scorePhase = SCORE_CHAIN_RETURN;
      }
      break;
    }

    case SCORE_CHAIN_RETURN: {
      if (fabs(chainTarget - chainbar_motors.get_position()) < CHAIN_TOLERANCE) {
        // Set target directly to 0 instead of using statesLow[0]
        target = 0; 
        liftPhase = LIFT_MOVING;   // straight move down, already hopped
        scorePhase = SCORE_DESCENDING;
      }
      break;
    }

    case SCORE_DESCENDING: {
      if (fabs(target - liftbar_motors.get_position()) < LIFT_TOLERANCE) {
        scorePhase = SCORE_IDLE;
      }
      break;
    }

    default: break;  // SCORE_IDLE — nothing to do
  }
}

const int SCORE_TIMEOUT_MS = 3000;  // give up and move on after this long

void score_and_wait(int stateIndex) {
  startScoreSequence(stateIndex);
  int waited = 0;
  while (scorePhase != SCORE_WAIT_UNCLAMP && waited < SCORE_TIMEOUT_MS) {
    pros::delay(10);
    waited += 10;
  }
}

void wait_for_score_finish() {
  int waited = 0;
  while (scorePhase != SCORE_IDLE && waited < SCORE_TIMEOUT_MS) {
    pros::delay(10);
    waited += 10;
  }
}

















/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
   chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
      {"Main Auto\n\nThis is the main autonomous routine for the robot.", mainAuto},
      {"Main Skills\n\nThis is the main skills routine for the robot.", mainSkill},
      {"Skills Two\n\nThis is the second skills routine for the robot.", skills_two},
  });

  liftbar_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  chainbar_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  resetLiftAndChain();

  pros::Task liftControalTask([]{
    while (true) {
      liftControl();
      pros::delay(10);
    } 
  });

  pros::Task chainControlTask([]{
    while (true) {
      chainControl();
      pros::delay(10);
    }
  });

  pros::Task scoreMacroTask([]{
    while (true) {
      scoreMacroUpdate();
      pros::delay(10);
    }
  });



  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
  // . . .
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  
  target = liftbar_motors.get_position();
  chainTarget = chainbar_motors.get_position();
  pendingTarget = target;
  liftPhase = LIFT_MOVING;
  scorePhase = SCORE_IDLE;

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants

   // if (master.get_digital(DIGITAL_X)) {             
   //   chassis.pid_tuner_toggle();

   // }

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
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
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade


    


    //Config controler buttons


    piston.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1));

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      highGoalMode = !highGoalMode;
    }

     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
     
      startScoreSequence(0);


    }


    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
     
      startScoreSequence(1);
    }


    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
     
      startScoreSequence(2);
    }


    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
     
      startScoreSequence(3);
   
    }


    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
     
      startScoreSequence(4);
   
    }


    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
     
      startScoreSequence(5);
   
    }






   

   

  

    if (ez::as::page_blank_is_on(1)) {
      
      ez::screen_print(piston.get() ? "Piston: ON" : "Piston: OFF", 2);
      ez::screen_print(highGoalMode ? "Mode: HIGH GOAL" : "Mode: LOW GOAL", 3);

    }

 double current_distance = distance_sensor.get();

    static bool auto_has_fired = false;
    static bool driver_overrode = false;
    static bool initial_check_done = false;

    // ==========================================
    // DYNAMIC SPEED & THRESHOLD (VIA JOYSTICK)
    // ==========================================
    // Read joystick throttle (Change to ANALOG_LEFT_Y if your drive is on the left stick)
    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); 
    double current_speed_effort = abs(throttle); // 0 to 127

    // Base distance is 30mm when stopped. At full stick (127), it scales up to 50mm.
    double dynamic_threshold = 30.0 + (current_speed_effort / 127.0) * 20.0;

    // Single definition for object detection
    bool object_detected = (current_distance > 0 && current_distance < dynamic_threshold);

    // Startup safety: if a pin is already inside when code boots up, don't auto-clamp it
    if (!initial_check_done) {
        if (object_detected) {
            auto_has_fired = true;
        }
        initial_check_done = true;
    }

    // 1. MANUAL L1 CONTROL (Absolute Highest Priority)
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        pistonClamped = !pistonClamped;    // Toggle your tracking variable
        piston.set(pistonClamped);         // Send to physical piston
        driver_overrode = true;            // Block auto-clamp from fighting you
    }
    // 2. AUTO-CLAMP LOGIC
    else if (object_detected && !auto_has_fired && !driver_overrode) {
        pistonClamped = true;              // Set to clamped state
        piston.set(false);                 // Trigger physical clamp (using false based on your setup)
        auto_has_fired = true;             // Lock it so it doesn't spam commands
    }

    // 3. RESET WHEN PIN IS REMOVED
    if (current_distance > 40 || current_distance == 0 || current_distance == 9999) {
        auto_has_fired = false;
        driver_overrode = false;
    }
    
    



    


    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

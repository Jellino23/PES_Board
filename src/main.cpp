//Test Test, de Pedro isch fett und de Maik blöd
//Test Test, dLeti isch chlii
//da isch korrekt

#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "UltrasonicSensor.h"
#include "DCMotor.h"
#include "FastPWM.h"
#include "Servo.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition below

// main runs as an own thread
int main()
{
    // set up states for state machine
    enum RobotState {
        INITIAL,
        PLATFORM,
        ROPEPREPARE,
        ROPE,
        OBSTACLEPREPARE,
        OBSTACLE,
        SLEEP,
        EMERGENCY
    } robot_state = RobotState::INITIAL;
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an aditional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // servo
    Servo servo_D0(PB_D0);
    Servo servo_D1(PB_D1);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // reely S0090 
    //ANPASSEN
    float servo_D0_ang_min = 0.0325f;
    float servo_D0_ang_max = 0.1175f;
    float servo_D1_ang_min = 0.0325f;
    float servo_D1_ang_max = 0.1175f;

    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);

    // EVTL ANPASSEN
    // default acceleration of the servo motion profile is 1.0e6f
    servo_D0.setMaxAcceleration(0.3f);
    servo_D1.setMaxAcceleration(0.3f);

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    // 6.0f V if you only use one battery pack

    // Motor M1
    const float gear_ratio_M1 = 100.0f; // gear ratio
    const float kn_M1 = 140.0f / 12.0f;  // motor constant [rpm/V]
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);
    // limit max. velocity to half physical possible velocity
    motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    motor_M1.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.5f);

    // Motor M2
    const float gear_ratio_M2 = 100.0f; // gear ratio
    const float kn_M2 = 140.0f / 12.0f;  // motor constant [rpm/V]
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);
    // limit max. velocity to half physical possible velocity
    motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    motor_M2.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);

    // Motor M3
    const float gear_ratio_M3 = 100.0f; // gear ratio
    const float kn_M3 = 140.0f / 12.0f;  // motor constant [rpm/V]
    DCMotor motor_M3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_M3, kn_M3, voltage_max);
    // limit max. velocity to half physical possible velocity
    motor_M3.setMaxVelocity(motor_M3.getMaxPhysicalVelocity() * 0.5f);
    // enable the motion planner for smooth movements
    motor_M3.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M3.setMaxAcceleration(motor_M3.getMaxAcceleration() * 0.5f);

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
                    enable_motors = 1;
                    if (!servo_D0.isEnabled())
                        servo_D0.enable();
                    if (!servo_D1.isEnabled())
                        servo_D1.enable();
                    break;
                }
                case RobotState::PLATFORM: {
                    motor_M1.setVelocity(motor_M1.getMaxVelocity());
                    motor_M2.setVelocity(motor_M2.getMaxVelocity());
                    break;
                }
                case RobotState::ROPEPREPARE: {
                    //ANPASSEN
                    servo_D0.setPulseWidth(1.0f);
                    servo_D1.setPulseWidth(1.0f);
                    break;
                }
                case RobotState::ROPE: {
                    motor_M1.setVelocity(motor_M1.getMaxVelocity());
                    motor_M2.setVelocity(motor_M2.getMaxVelocity());
                    break;
                }
                case RobotState::OBSTACLEPREPARE: {
                    //ANPASSEN
                    servo_D0.setPulseWidth(0.0f);
                    servo_D1.setPulseWidth(0.0f);
                    break;
                }
                case RobotState::OBSTACLE: {

                    break;
                }
                case RobotState::SLEEP: {

                    break;
                }
                case RobotState::EMERGENCY: {
                    //steppermotor zurück auf 0.0f
                    //motoren ausschalten

                    break;
                }
                default: {

                    break; // do nothing
                }
            }
            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
                servo_D0.disable();
                servo_D1.disable();
                enable_motors = 0;
                //us_distance_cm = 0.0f;
                robot_state = RobotState::INITIAL;

            }
        }

        // toggling the user led
        user_led = !user_led;

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}

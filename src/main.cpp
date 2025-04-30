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
#include "LineFollower.h"
#include "IRSensor.h"

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
    float servo_D0_ang_min = 0.0310f;
    float servo_D0_ang_max = 0.118f;
    float servo_D1_ang_min = 0.0315f;
    float servo_D1_ang_max = 0.122f;

    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);

    // EVTL ANPASSEN
    // default acceleration of the servo motion profile is 1.0e6f
    servo_D0.setMaxAcceleration(0.3f);
    servo_D1.setMaxAcceleration(0.3f);

    // variables to move the servo, this is just an example
    float servo_input_left = 0.5f;
    float servo_input_right = 0.5f;
    int servo_counter_left = 0; // define servo counter, this is an additional variable
                           // used to command the servo
    int servo_counter_right = 0;
    const int loops_per_seconds = static_cast<int>(ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    //Variablen wo dass das Gewicht ist
    float weight_down_left = 0.63f;
    float weight_up_left = 0.05f;
    float weight_down_right = 0.2f;
    float weight_up_right = 0.75f;

    servo_input_left = weight_up_left;
    servo_input_right = weight_up_right;

    // create object to enable power electronics for the dc motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    // 6.0f V if you only use one battery pack

    // Motor M1
    const float gear_ratio_M1 = 100.0f; // gear ratio
    const float kn_M1 = 140.0f / 12.0f;  // motor constant [rpm/V]
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);
    // limit max. velocity to half physical possible velocity
    motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 0.28f);
    // enable the motion planner for smooth movements
    // //motor_M1.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.28f);

    // Motor M2
    const float gear_ratio_M2 = 100.0f; // gear ratio
    const float kn_M2 = 140.0f / 12.0f;  // motor constant [rpm/V]
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);
    // limit max. velocity to half physical possible velocity
    motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.28f);
    // enable the motion planner for smooth movements
    //motor_M2.enableMotionPlanner();
    // limit max. acceleration to half of the default acceleration
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.28f);


    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
    // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
    // is a defined potential

    // ultra sonic sensor
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 200.0f;

    int platform = 1;
    //linefollower
    int ir_sees_ground = 0;
    float rotationBeforeRopeM1 = 0.0f;
    float rotationBeforeRopeM2 = 0.0f;
    float diff_Rot_M1 = 0.0f;
    float diff_Rot_M2 = 0.0f;

    // line follower, tune max. vel rps to your needs
    const float d_wheel = 0.046f; // wheel diameter in meters
    const float b_wheel = 0.153f;  // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.02f; // distance from wheel axis to leds on sensor bar / array in meters
    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M2.getMaxPhysicalVelocity());
    
    // nonlinear controller gains, tune to your needs (linefollower)
    const float Kp = 1.2f * 2.0f;
    const float Kp_nl = 1.2f * 17.0f;
    lineFollower.setRotationalVelocityGain(Kp, Kp_nl);

    // ir distance sensor with average filter and implicit calibration

    float distance_to_ground = 7.0f;
    float ir_distance_cm = 0.0f;
    float ir_distance_cm_read = 0.0f;
    IRSensor ir_sensor(PC_2);                      // before the calibration the read function will return the averaged mV value
    //ANPASSEN
    ir_sensor.setCalibration(11821.2086f, -122.1091f); // after the calibration the read function will return the calibrated value

    Timer initial_state_timer;
    bool initial_timer_started = false;

    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            
            ir_distance_cm_read = ir_sensor.readcm();
            ir_distance_cm = ir_distance_cm_read - 3.0;

            //read distance with us_sensor
            const float us_distance_cm_candidate = us_sensor.read();
            if (us_distance_cm_candidate > 0.0f){        //only valid measurments are accepted
                us_distance_cm = us_distance_cm_candidate;
            }
            switch (robot_state) {
                case RobotState::INITIAL: {
                    printf("INITIAL\n");
                    // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
                    enable_motors = 1;
                    if (!servo_D0.isEnabled())
                        servo_D0.enable(weight_up_left);
                    if (!servo_D1.isEnabled())
                        servo_D1.enable(weight_up_right);
                    //Linefollower sieht Line? -->

                    if(!initial_timer_started){
                        initial_state_timer.start();
                        initial_timer_started = true;
                    }

                    if (duration_cast<seconds>(initial_state_timer.elapsed_time()).count() >= 3) {
                        // Setze den Timer zurück für den nächsten Durchlauf
                        initial_state_timer.reset();
                        initial_timer_started = false;
                        robot_state = RobotState::PLATFORM;
                    }

                    break;
                }
                case RobotState::PLATFORM: {
                    printf("PLATFORM\n");

                    servo_D0.setPulseWidth(weight_up_left);
                    servo_D1.setPulseWidth(weight_up_right);
                    //Auf der Startplattform
                    if(platform == 1){
                        motor_M1.setVelocity(lineFollower.getLeftWheelVelocity());
                        motor_M2.setVelocity(lineFollower.getRightWheelVelocity());
                    }
                    if(platform == 2){
                        motor_M1.setVelocity(motor_M1.getMaxVelocity());
                        motor_M2.setVelocity(motor_M2.getMaxVelocity());
                    }
                    if(ir_sees_ground == 0){
                        rotationBeforeRopeM1 = motor_M1.getRotation();
                        rotationBeforeRopeM2 = motor_M2.getRotation();
                    }

                    //if(us_distance_cm < 25 && us_distance_cm > 20){
                    if((ir_distance_cm > distance_to_ground) || (ir_sees_ground == 1)){ 
                        platform = 2;
                        ir_sees_ground = 1;
                        diff_Rot_M1 = (motor_M1.getRotation() - rotationBeforeRopeM1);
                        diff_Rot_M2 = (motor_M2.getRotation() - rotationBeforeRopeM2);

                        //motor_M1.setVelocity(motor_M1.getMaxVelocity());
                        //motor_M2.setVelocity(motor_M2.getMaxVelocity());
                        
                        if( (diff_Rot_M1 >= 1.0f) && (diff_Rot_M2 >= 1.0f)){
                            ir_sees_ground = 0;
                            
                            robot_state = RobotState::ROPEPREPARE;
                        }
                        /*if (((us_distance_cm < 115 && us_distance_cm > 95)== false)) {   //hier sagen das  noch vorgefahren werden soll
                            motor_M1.setVelocity(motor_M1.getMaxVelocity());                                          //vieleicht besser wenn eine weitere plattform einfügen für den schluss  
                            motor_M2.setVelocity(motor_M2.getMaxVelocity()); 
                        }
                        else{
                            robot_state = RobotState::ROPEPREPARE;
                        }*/
                    }
                    /*if(us_distance_cm < 15){
                        motor_M1.setVelocity(0.0f);                                          //vieleicht besser wenn eine weitere plattform einfügen für den schluss  
                        motor_M2.setVelocity(0.0f);
                    }*/
                    break;
                }
                case RobotState::ROPEPREPARE: {
                    //ANPASSEN
                    printf("ROPEPREPARE\n");
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    // calculate inputs for the servos for the next cycle
                    if ((servo_input_right > 0.0f) &&                     // constrain servo_input to be < 1.0f
                        (servo_counter_right % loops_per_seconds == 0) && // true if servo_counter is a multiple of loops_per_second
                        (servo_counter_right != 0))                       // avoid servo_counter = 0
                        servo_input_right -= 0.1f;
                    servo_counter_right++;

                    if ((servo_input_left < 1.0f) &&                     // constrain servo_input to be < 1.0f
                        (servo_counter_left % loops_per_seconds == 0) && // true if servo_counter is a multiple of loops_per_second
                        (servo_counter_left != 0))                       // avoid servo_counter = 0
                        servo_input_left += 0.1f;
                    servo_counter_left++;

                    if(servo_input_right < weight_down_right && servo_counter_left > weight_down_left){
                        servo_input_right = weight_down_right;
                        servo_input_left = weight_down_left;
                        robot_state = RobotState::ROPE;
                    }
                    servo_D0.setPulseWidth(servo_input_left);
                    servo_D1.setPulseWidth(servo_input_right);
                    break;
                }
                case RobotState::ROPE: {
                    printf("ROPE\n");
                    motor_M1.setVelocity(motor_M1.getMaxVelocity());
                    motor_M2.setVelocity(motor_M2.getMaxVelocity());

                    if(ir_distance_cm < distance_to_ground){
                        robot_state = RobotState::OBSTACLEPREPARE;
                    }

                    break;
                }
                case RobotState::OBSTACLEPREPARE: {
                    printf("OBSTACLEPREPARE\n");
                    /*
                    Problem: Motor hört nicht auf zu drehen, wie bringt man motor zum stoppen?
                    */
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    
                    if ((servo_input_left > 0.0f) &&                     // constrain servo_input to be < 1.0f
                        (servo_counter_left % loops_per_seconds == 0) && // true if servo_counter is a multiple of loops_per_second
                        (servo_counter_left != 0))                       // avoid servo_counter = 0
                        servo_input_left -= 0.1f;
                    servo_counter_left++;
                    
                    if ((servo_input_right < 1.0f) &&                     // constrain servo_input to be < 1.0f
                        (servo_counter_right % loops_per_seconds == 0) && // true if servo_counter is a multiple of loops_per_second
                        (servo_counter_right != 0))                       // avoid servo_counter = 0
                        servo_input_right += 0.1f;
                    servo_counter_right++;

                    if(servo_input_right > weight_up_right && servo_input_left < weight_up_left){
                        servo_input_left = weight_up_left;
                        servo_input_right = weight_up_right;
                        robot_state = RobotState::PLATFORM;
                        /*if(us_distance_cm < 95 && us_distance_cm > 85){           //gleiche masse wie in zeile 276
                             robot_state = RobotState::OBSTACLE;                                                 
                            }
                        if(us_distance_cm < 55 && us_distance_cm > 45){           //masse anpassen
                            robot_state = RobotState::PLATFORM;*/
                        }
                    servo_D0.setPulseWidth(servo_input_left);
                    servo_D1.setPulseWidth(servo_input_right);
                    break;
                    }
                case RobotState::OBSTACLE: {
                    printf("OBSTACLE\n");
                    motor_M1.setVelocity(motor_M1.getMaxVelocity());
                    motor_M2.setVelocity(motor_M2.getMaxVelocity());
                    if(us_distance_cm < 75 && us_distance_cm > 73){              //genauere Massangaben
                        robot_state = RobotState::ROPEPREPARE;
                    }
                    break;
                }
                /*case RobotState::SLEEP: {
                    printf("SLEEP\n");
                    break;
                }*/
                /*case RobotState::EMERGENCY: {
                    printf("EMERGENCY\n");
                    //steppermotor zurück auf 0.0f
                    //motoren ausschalten

                    // disable the motion planer and
                    // move to the initial position asap
                    // then reset the system
                    servo_D0.setMaxAcceleration(1.0f);
                    servo_D1.setMaxAcceleration(1.0f);
                    servo_D0.setPulseWidth(weight_up_left);
                    servo_D1.setPulseWidth(weight_up_right);

                    motor_M1.disableMotionPlanner();
                    motor_M1.setRotation(0.0f);
                    motor_M2.disableMotionPlanner();
                    motor_M2.setRotation(0.0f);
                    if (motor_M1.getRotation() < 0.1 && motor_M2.getRotation() < 0.1 )
                        toggle_do_execute_main_fcn();
                    

                    break;
                }*/
                default: {

                    break; // do nothing
                }
            }

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;
                // reset variables and objects
                led1 = 0;
                // reset variables and objects
                servo_D0.disable();
                servo_D1.disable();
                enable_motors = 0;
                us_distance_cm = 200.0f;

                ir_distance_cm = 0.0f;

                robot_state = RobotState::INITIAL;
            }
        }

        // toggling the user led
        user_led = !user_led;

        //printf("US distance cm: %f \n", us_distance_cm);
        // print to the serial terminal
        printf("IR distance cm: %f ", ir_distance_cm);
        printf("IR Sees Ground: %d ", ir_sees_ground);
        printf("beforeroopeM1: %f ", rotationBeforeRopeM1);
        printf("motor_M1 getRotation: %f ", motor_M1.getRotation());
        //printf("motor_M2 getRotation: %f ", motor_M2.getRotation());
        printf("diffrotation M1: %f ", diff_Rot_M1);
        printf("diffrotation M2: %f \n", diff_Rot_M2);
       

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
#include "mbed.h"

// PES Board Pin Mapping
#include "PESBoardPinMap.h"

// Treiber
#include "DebounceIn.h"
#include "UltrasonicSensor.h"
#include "DCMotor.h"
#include "FastPWM.h"
#include "Servo.h"
#include "LineFollower.h"
#include "IRSensor.h"

// Globale Variablen
bool do_execute_main_task = false; // Steuert die Ausführung der Hauptaufgabe (wird durch Benutzerbutton getoggelt)
bool do_reset_all_once = false;    // Wird verwendet, um Variablen einmalig zurückzusetzen

// Objekte für Benutzerbutton (blauer Button) auf dem Nucleo-Board
DebounceIn user_button(BUTTON1);   // DebounceIn-Objekt zur Auswertung des Benutzerbuttons
void toggle_do_execute_main_fcn(); // Funktion, die bei Button-Druck ausgefÃ¼hrt wird

// Hauptprogramm läuft als eigener Thread
int main()
{
    // Zustände für die Zustandsmaschine
    enum RobotState {
        INITIAL,          // Initialisierungszustand
        PLATFORM,         // Plattform-Phase
        ROPEPREPARE,      // Vorbereitung für Seil-Phase
        ROPE,             // Seil-Phase
        OBSTACLEPREPARE,  // Vorbereitung für Hindernis-Phase
        DANCE             // Tanz-Phase (Finale)
    } robot_state = RobotState::INITIAL;
    
    // Button-Funktion zuweisen
    user_button.fall(&toggle_do_execute_main_fcn);

    // Haupttask wird alle main_task_period_ms Millisekunden ausgeführt
    const int main_task_period_ms = 20; // Haupttask-Periodendauer in ms (50 Hz)
    Timer main_task_timer;              // Timer für die periodische Ausführung

    // LED auf dem Nucleo-Board
    DigitalOut user_led(LED1);

    // Zusätzliche LED
    DigitalOut led1(PB_9);

    // Servos initialisieren
    Servo servo_D0(PB_D0);
    Servo servo_D1(PB_D1);

    // Kalibrierungswerte für die Servos (minimale/maximale Pulsbreite)
    float servo_D0_ang_min = 0.0310f;
    float servo_D0_ang_max = 0.118f;
    float servo_D1_ang_min = 0.0315f;
    float servo_D1_ang_max = 0.122f;

    // Servos kalibrieren
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);

    // Maximale Beschleunigung der Servos einstellen
    servo_D0.setMaxAcceleration(0.3f);
    servo_D1.setMaxAcceleration(0.3f);

    // Servo-Steuerungsvariablen
    float servo_input_left = 0.5f;
    float servo_input_right = 0.5f;
    int servo_counter_left = 0;  // Zähler für Servobewegung
    int servo_counter_right = 0;
    const int loops_per_seconds = static_cast<int>(ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    // Positionen für die Gewichte (Servo-Winkel)
    float weight_down_left = 0.52f;
    float weight_up_left = 0.025f;
    float weight_dance_left = 0.2f;
    float weight_down_right = 0.3f;
    float weight_up_right = 0.8f;
    float weight_dance_right = 0.6f;

    // Initiale Servopositionen
    servo_input_left = weight_up_left;
    servo_input_right = weight_up_right;

    // Motorsteuerung aktivieren
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // Maximale Batteriespannung
    const float voltage_max = 12.0f;

    // Motor M1 konfigurieren
    const float gear_ratio_M1 = 100.0f; // Übersetzungsverhältnis
    const float kn_M1 = 140.0f / 12.0f; // Motorkonstante [rpm/V]
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);
    motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 0.28f); // Maximalgeschwindigkeit begrenzen
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.28f); // Beschleunigung begrenzen

    // Motor M2 konfigurieren (analog zu M1)
    const float gear_ratio_M2 = 100.0f;
    const float kn_M2 = 140.0f / 12.0f;
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);
    motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.28f);
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.28f);

    // Mechanischer Button
    DigitalIn mechanical_button(PC_5);
    mechanical_button.mode(PullUp); // PullUp-Widerstand aktivieren

    // Ultraschallsensor
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 0.0f;

    // Zustandsvariablen
    int platform = 1; // Plattform-Status (1 = Startplattform, 2 = Endplattform)
    int ir_sees_ground = 0; // IR-Sensor erkennt Boden
    float rotationBeforeRopeM1 = 0.0f; // Motorrotation vor Seilphase
    float rotationBeforeRopeM2 = 0.0f;
    float rotationBeforePlatM1 = 0.0f; // Motorrotation vor Plattformphase
    float rotationBeforePlatM2 = 0.0f;
    float diff_Rot_M1 = 0.0f; // Rotationsdifferenz
    float diff_Rot_M2 = 0.0f;
    float diff_Rot_M1_end = 0.0f; // Rotationsdifferenz für Endphase
    float diff_Rot_M2_end = 0.0f;
    
    int endplat_get_rotation = 1; // Flag für Rotationsmessung auf Endplattform

    int dance_servo_down = 0; // Status für Servos im Tanz
    int finish_turn = 1;      // Flag für Abschlussdrehung
    int finish_get_rotation = 1; // Flag für Rotationsmessung im Tanz

    // LineFollower konfigurieren
    const float d_wheel = 0.046f; // Raddurchmesser [m]
    const float b_wheel = 0.153f;  // Achsabstand [m]
    const float bar_dist = 0.02f; // Abstand von Radachse zu LED-Sensorleiste [m]
    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_M2.getMaxPhysicalVelocity());
    
    // Reglerparameter für LineFollower
    const float Kp = 1.2f * 2.0f;
    const float Kp_nl = 1.2f * 17.0f;
    lineFollower.setRotationalVelocityGain(Kp, Kp_nl);

    // IR-Sensor mit Filter und Kalibrierung
    float distance_to_ground = 10.0f; // Schwellwert für Bodenerkennung [cm]
    float ir_distance_cm = 0.0f;     // Gefilterte IR-Messung
    float ir_distance_cm_read = 0.0f; // Rohmessung
    IRSensor ir_sensor(PC_2);
    ir_sensor.setCalibration(11821.2086f, -122.1091f); // Kalibrierungsparameter

    // Timer für verschiedene Zustände
    Timer initial_state_timer;
    bool initial_timer_started = false;
    Timer initial_Servo_Zeitgeber;
    bool initial_Servo_Zeitgeber_started = false;
    Timer total_time_gone;
    bool total_time_gone_started = false;

    // Timer starten
    main_task_timer.start();

    // Hauptschleife
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            // Visuelles Feedback, dass Haupttask läuft
            led1 = 1;
            
            // IR-Sensor auslesen
            ir_distance_cm_read = ir_sensor.readcm();
            ir_distance_cm = ir_distance_cm_read - 3.0;

            // Ultraschallsensor auslesen (nur gültige Werte Übernehmen)
            float us_distance_cm_candidate = 0.0f;
            us_distance_cm_candidate = us_sensor.read();
            if (us_distance_cm_candidate > 0.0f) {
                us_distance_cm = us_distance_cm_candidate;
            }

            // Zustandsmaschine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    printf("INITIAL\n");
                    // Hardware aktivieren
                    enable_motors = 1;
                    if (!servo_D0.isEnabled())
                        servo_D0.enable(weight_up_left);
                    if (!servo_D1.isEnabled())
                        servo_D1.enable(weight_up_right);

                    // Timer für Initialisierungsphase starten
                    if(!initial_timer_started) {
                        initial_state_timer.start();
                        initial_timer_started = true;
                    }

                    // Gesamtzeit-Timer starten
                    if(!total_time_gone_started) {
                        total_time_gone.start();
                        total_time_gone_started = true;
                    }

                    // Nach 3 Sekunden in nächsten Zustand wechseln
                    if (duration_cast<seconds>(initial_state_timer.elapsed_time()).count() >= 3) {
                        initial_state_timer.stop();
                        initial_state_timer.reset();
                        initial_timer_started = false;
                        robot_state = RobotState::PLATFORM;
                    }
                    break;
                }
                case RobotState::PLATFORM: {
                    printf("PLATFORM\n");

                    // Servos in Ausgangsposition
                    servo_D0.setPulseWidth(weight_up_left);
                    servo_D1.setPulseWidth(weight_up_right);

                    // Auf Startplattform: LineFollower nutzen
                    if(platform == 1) {
                        motor_M1.setVelocity(lineFollower.getLeftWheelVelocity());
                        motor_M2.setVelocity(lineFollower.getRightWheelVelocity());
                    }
                    
                    // Auf Endplattform: Geradeaus fahren
                    if(platform == 2) {
                        motor_M1.setVelocity(motor_M1.getMaxVelocity());
                        motor_M2.setVelocity(motor_M2.getMaxVelocity());

                        // Nach 40 Sekunden Endplattform-Phase
                        if (duration_cast<seconds>(total_time_gone.elapsed_time()).count() >= 40) {
                            if(endplat_get_rotation == 1) {
                                // Rotationen speichern
                                rotationBeforePlatM1 = motor_M1.getRotation();
                                rotationBeforeRopeM2 = motor_M2.getRotation();
                                endplat_get_rotation = 0;
                            }
                            
                            // Rotationsdifferenz berechnen
                            diff_Rot_M1_end = (motor_M1.getRotation() - rotationBeforePlatM1);
                            diff_Rot_M2_end = (motor_M2.getRotation() - rotationBeforePlatM2);

                            // Wenn genug weit gefahren und Hindernis erkannt: Tanzphase starten
                            if( (diff_Rot_M1_end >= 3.0f) && (diff_Rot_M2_end >= 3.0f) && us_distance_cm < 25) {
                                robot_state = RobotState::DANCE;
                            }
                        }
                    }

                    // IR-Sensor erkennt Boden?
                    if(ir_sees_ground == 0) {
                        rotationBeforeRopeM1 = motor_M1.getRotation();
                        rotationBeforeRopeM2 = motor_M2.getRotation();
                    }

                    // Plattformwechsel erkennen
                    if((ir_distance_cm > distance_to_ground) || (ir_sees_ground == 1)) { 
                        platform = 2;
                        ir_sees_ground = 1;
                        diff_Rot_M1 = (motor_M1.getRotation() - rotationBeforeRopeM1);
                        diff_Rot_M2 = (motor_M2.getRotation() - rotationBeforeRopeM2);

                        // Wenn genug weit gefahren: Vorbereitung Seilphase
                        if( (diff_Rot_M1 >= 1.3f) && (diff_Rot_M2 >= 1.3f)) {
                            ir_sees_ground = 0;
                            robot_state = RobotState::ROPEPREPARE;
                        }
                    }
                    break;
                }
                case RobotState::ROPEPREPARE: {
                    printf("ROPEPREPARE\n");
                    // Motoren stoppen
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    
                    // Servos langsam in Position bringen 
                    if ((servo_input_right > 0.0f) && 
                        (servo_counter_right % loops_per_seconds == 0) && 
                        (servo_counter_right != 0))
                        servo_input_right -= 0.1f;
                    servo_counter_right++;

                    if ((servo_input_left < 1.0f) && 
                        (servo_counter_left % loops_per_seconds == 0) && 
                        (servo_counter_left != 0))
                        servo_input_left += 0.1f;
                    servo_counter_left++;

                    // Wenn Servos in Position: Seilphase starten
                    if(servo_input_right < weight_down_right && servo_counter_left > weight_down_left) {
                        servo_input_right = weight_down_right;
                        servo_input_left = weight_down_left;
                        robot_state = RobotState::ROPE;
                    }

                    // Servos ansteuern
                    servo_D0.setPulseWidth(servo_input_left);
                    servo_D1.setPulseWidth(servo_input_right);
                    break;
                }
                case RobotState::ROPE: {
                    printf("ROPE\n");
                    // Motoren mit reduzierter Geschwindigkeit laufen lassen
                    motor_M1.setVelocity(motor_M1.getMaxVelocity() * 0.7);
                    motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.7);
                    
                    // Wenn Boden erkannt: Vorbereitung Hindernisphase
                    if(ir_distance_cm < distance_to_ground) {
                        robot_state = RobotState::OBSTACLEPREPARE;
                    }
                    break;
                }
                case RobotState::OBSTACLEPREPARE: {
                    printf("OBSTACLEPREPARE\n");
                    // Motoren stoppen
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    
                    // Servos langsam in Ausgangsposition bringen
                    if ((servo_input_left > 0.0f) && 
                        (servo_counter_left % loops_per_seconds == 0) && 
                        (servo_counter_left != 0))
                        servo_input_left -= 0.1f;
                    servo_counter_left++;
                    
                    if ((servo_input_right < 1.0f) && 
                        (servo_counter_right % loops_per_seconds == 0) && 
                        (servo_counter_right != 0))
                        servo_input_right += 0.1f;
                    servo_counter_right++;
                    
                    // Wenn Servos in Position: Wartezeit einhalten
                    if(servo_input_right > weight_up_right && servo_input_left < weight_up_left) {
                        servo_input_left = weight_up_left;                  
                        servo_input_right = weight_up_right;
                        
                        if (!initial_Servo_Zeitgeber_started) {
                            initial_Servo_Zeitgeber.reset();
                            initial_Servo_Zeitgeber.start();
                            initial_Servo_Zeitgeber_started = true;
                        }
                        
                        // Nach 0.75 Sekunden zurück zur Plattformphase
                        if (duration_cast<seconds>(initial_Servo_Zeitgeber.elapsed_time()).count() >= 0.75) {
                            initial_Servo_Zeitgeber.stop();
                            initial_Servo_Zeitgeber.reset();
                            initial_Servo_Zeitgeber_started = false;
                            robot_state = RobotState::PLATFORM;
                        }
                    }
                    
                    // Servos ansteuern
                    servo_D0.setPulseWidth(servo_input_left);
                    servo_D1.setPulseWidth(servo_input_right);
                    break;
                }
                case RobotState::DANCE: {
                    printf("DANCE\n");
                    // Motoren stoppen
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);

                    // Abschlussdrehung durchführen
                    if(finish_turn == 1) {
                        if (finish_get_rotation == 1) {
                            rotationBeforeRopeM1 = motor_M1.getRotation();
                            rotationBeforeRopeM2 = motor_M2.getRotation();
                            finish_get_rotation = 0;
                        }
                        // Drehung (M1 rückwärts, M2 vorwärts)
                        motor_M1.setVelocity(motor_M1.getMaxVelocity() * (-0.7));
                        motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.7);
                        diff_Rot_M1 = (motor_M1.getRotation() - rotationBeforeRopeM1);
                        diff_Rot_M2 = (motor_M2.getRotation() - rotationBeforeRopeM2);

                        // Wenn Drehung abgeschlossen: Servos bewegen
                        if (diff_Rot_M2 >= 3.95f) {
                            motor_M1.setVelocity(0.0f);
                            motor_M2.setVelocity(0.0f);
                            finish_turn = 0;
                            dance_servo_down = 1;
                        }
                    }

                    // Servos nach unten bewegen
                    if(dance_servo_down == 1) {
                        if ((servo_input_right > 0.0f) && 
                            (servo_counter_right % loops_per_seconds == 0) && 
                            (servo_counter_right != 0))
                            servo_input_right -= 0.1f;
                        servo_counter_right++;

                        if ((servo_input_left < 1.0f) && 
                            (servo_counter_left % loops_per_seconds == 0) && 
                            (servo_counter_left != 0))
                            servo_input_left += 0.1f;
                        servo_counter_left++;
                        
                        // Wenn Servos in Position: nächste Phase
                        if(servo_input_right < weight_dance_right && servo_counter_left > weight_dance_left) {
                            servo_input_right = weight_dance_right;
                            servo_input_left = weight_dance_left;
                            dance_servo_down = 2;
                        }
                    }

                    // Servos zurück nach oben bewegen
                    if(dance_servo_down == 2) {
                        if ((servo_input_left > 0.0f) && 
                            (servo_counter_left % loops_per_seconds == 0) && 
                            (servo_counter_left != 0))
                            servo_input_left -= 0.1f;
                        servo_counter_left++;
                        
                        if ((servo_input_right < 1.0f) && 
                            (servo_counter_right % loops_per_seconds == 0) && 
                            (servo_counter_right != 0))
                            servo_input_right += 0.1f;
                        servo_counter_right++;
                        
                        // Wenn Servos oben: Position halten
                        if(servo_input_right > weight_up_right && servo_input_left < weight_up_left) {
                            servo_input_left = weight_up_left;                  
                            servo_input_right = weight_up_right;
                        }
                    }

                    // Servos ansteuern
                    servo_D0.setPulseWidth(servo_input_left);
                    servo_D1.setPulseWidth(servo_input_right);
                    break;
                }
                default: {
                    break; // nichts tun
                }
            }

        } else {
            // Reset-Routine (wird einmal ausgeführt, wenn Haupttask deaktiviert wird)
            if (do_reset_all_once) {
                do_reset_all_once = false;
                // Alle Komponenten deaktivieren
                led1 = 0;
                servo_D0.disable();
                servo_D1.disable();
                enable_motors = 0;
                us_distance_cm = 200.0f;

                // Variablen zurücksetzen
                ir_distance_cm = 0.0f;
                total_time_gone.stop();
                total_time_gone.reset();
                total_time_gone_started = false;
                endplat_get_rotation = 1;

                // Zurück in Initialzustand
                robot_state = RobotState::INITIAL;
            }
        }

        // User-LED toggeln
        user_led = !user_led;

        // Debug-Ausgaben
        // printf("US distance cm: %f \n", us_distance_cm);
        // print to the serial terminal
        // printf("IR distance cm: %f ", ir_distance_cm);
        // printf("IR Sees Ground: %d ", ir_sees_ground);
        // printf("beforeroopeM1: %f ", rotationBeforeRopeM1);
        // printf("motor_M1 getRotation: %f ", motor_M1.getRotation());
        //printf("motor_M2 getRotation: %f ", motor_M2.getRotation());
        // printf("diffrotation M1: %f ", diff_Rot_M1);
        // printf("diffrotation M2: %f \n", diff_Rot_M2);

        // Restliche Zeit bis zum nächsten Zyklus warten
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

// Funktion zum Umschalten des Haupttask-Status
void toggle_do_execute_main_fcn()
{
    // Haupttask-Status umschalten
    do_execute_main_task = !do_execute_main_task;
    // Reset-Flag setzen, wenn Haupttask aktiviert wird
    if (do_execute_main_task)
        do_reset_all_once = true;
}
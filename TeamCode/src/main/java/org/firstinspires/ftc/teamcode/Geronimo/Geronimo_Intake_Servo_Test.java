/* This code was used during the Freight Frenzy season to test LG's arm servo positions.
It uses a SCALE factor to incrementally move the servo to allow for precise positioning.
Code was developed by Jacob Lemanski and previously list in the menu as "Jacob Servo Test"
*/

/* Code developed to test the servos on Geronimo.  Instead of using fixed step increments
which may not be as accurate as needed for precise mechanism placement, code uses joysticks and scaling
factor to define servo movement step.

Triangle + Left Stick Y = move both smallArmHangerServos
Circle + Left Stick Y = move intakeBoxRotaterServo (rotate urchin)
Square + Left Stick Y = move clawServo
X + Left Stick Y = move both swiper servos
Left Bumper = Open urchinServo
Right Bumper = Close urchinServo
Left Trigger = Lock Hook
Right Trigger = Release (Unlock) Hook

  Y            Triangle
X   B     Square      Circle
  A               X
Note: Servos initialize to midpoint of servo travel
*/

package org.firstinspires.ftc.teamcode.Geronimo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Config // added for FTC dashboard usage
@Disabled
@TeleOp(name = "Geronimo Intake Servo Test")
public class Geronimo_Intake_Servo_Test extends LinearOpMode {
    static final double SCALE   = 0.001;  // Joystick scaling for servo increment value
    static final double MAX_POS = 1.0;    // Maximum rotational position
    static final double MIN_POS = 0.0;    // Minimum rotational position
    static final double INIT_POS = 0.5;   // Start at halfway position

    // Define class members
    Servo clawServo;
    Servo intakeBoxRotaterServo; // rotate urchin
    Servo smallArmHangerLeftServo;
    Servo smallArmHangerRightServo;
    Servo urchinServo;  //open close urchin
    Servo swiperServo;  //original swiper (right side)
    Servo swiper2;      // added swiper (left side)
    Servo lockServo1;   // TODO add to robot configuration file
    Servo lockServo2;   // TODO add to robot configuration file

    double claw_position = 0;         // claw closed
    double urchin_claw_position = 1;  // urchin closed
    double urchin_rotate_position = INIT_POS;
    double swiper_position = INIT_POS;
    double small_rotator_position = INIT_POS;
    double lock_position = 0;       // release position

    @Override
    public void runOpMode() {

        // Change the text in quotes to match any servo name on your robot.
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        intakeBoxRotaterServo = hardwareMap.get(Servo.class, "intakeRotater");
        smallArmHangerLeftServo = hardwareMap.get(Servo.class, "intakeHangerLeft");
        smallArmHangerRightServo = hardwareMap.get(Servo.class, "intakeHangerRight");
        urchinServo = hardwareMap.get(Servo.class, "urchinServo");
        swiperServo = hardwareMap.get(Servo.class, "swiper");
        swiper2 = hardwareMap.get(Servo.class, "swiper2");
        //lockServo1 = hardwareMap.get(Servo.class, "lock1"); TODO uncomment after config file update
        //lockServo2 = hardwareMap.get(Servo.class, "lock2"); TODO uncomment after config file update

        // Set servo directions (useful for paired servos on opposite sides of arm)
        smallArmHangerLeftServo.setDirection(Servo.Direction.REVERSE);
        smallArmHangerRightServo.setDirection(Servo.Direction.FORWARD);
        swiperServo.setDirection(Servo.Direction.FORWARD);
        swiper2.setDirection(Servo.Direction.REVERSE);
        //lockServo1.setDirection(Servo.Direction.FORWARD); TODO uncomment after config file update
        //lockServo2.setDirection(Servo.Direction.REVERSE); TODO uncomment after config file update

        // Move servos to init position (midpoint of servo travel)
        clawServo.setPosition(claw_position); // init closed
        urchinServo.setPosition(urchin_claw_position); // init closed
        intakeBoxRotaterServo.setPosition(INIT_POS);
        smallArmHangerLeftServo.setPosition(INIT_POS);
        smallArmHangerRightServo.setPosition(INIT_POS);
        swiperServo.setPosition(INIT_POS);
        swiper2.setPosition(INIT_POS);
        //lockServo1.setPosition(lock_position); TODO uncomment after config file update
        //lockServo2.setPosition(lock_position); TODO uncomment after config file update


        // Wait for the start button
        telemetry.addData(">", "Press Start to test servo movement.");
        telemetry.update();
        waitForStart();

        // Test servo positions until stop pressed.
        while (opModeIsActive()) {
            //**********************************************
            // Rotate Small Arms -- Triangle + Left Stick Y
            //**********************************************
            if (gamepad1.left_stick_y != 0 && gamepad1.triangle) {
                // Keep stepping until servo reaches the min/max value.
                small_rotator_position += gamepad1.left_stick_y * SCALE;

                if (small_rotator_position >= MAX_POS) {
                    small_rotator_position = MAX_POS;
                }
                if (small_rotator_position <= MIN_POS) {
                    small_rotator_position = MIN_POS;
                }
                smallArmHangerLeftServo.setPosition(small_rotator_position);
                smallArmHangerRightServo.setPosition(small_rotator_position);
            }
            //**********************************************
            // Rotate Urchin -- Circle + Left Stick Y
            //**********************************************
            if (gamepad1.left_stick_y != 0 && gamepad1.circle) {
                // Keep stepping until servo reaches the min/max value.
                urchin_rotate_position += gamepad1.left_stick_y * SCALE;

                if (urchin_rotate_position >= MAX_POS) {
                    urchin_rotate_position = MAX_POS;
                }
                if (urchin_rotate_position <= MIN_POS) {
                    urchin_rotate_position = MIN_POS;
                }
                intakeBoxRotaterServo.setPosition(urchin_rotate_position);
            }
            //**********************************************
            // Open/Close Claw -- Square + Left Stick Y
            //**********************************************
            if (gamepad1.left_stick_y != 0 && gamepad1.square) {
                // Keep stepping until servo reaches the min/max value.
                claw_position += gamepad1.left_stick_y * SCALE;

                if (claw_position >= MAX_POS) {
                    claw_position = MAX_POS;
                }
                if (claw_position <= MIN_POS) {
                    claw_position = MIN_POS;
                }
                clawServo.setPosition(claw_position);
            }
            //**********************************************
            // Open Urchin -- Left Bumper Open
            //**********************************************
            if (gamepad1.left_bumper) {
                urchin_claw_position = 0;
                urchinServo.setPosition(urchin_claw_position);
            }
            //**********************************************
            // Close Urchin -- Right Bumper Close
            //**********************************************
            if (gamepad1.right_bumper) {
                urchin_claw_position = 1;
                urchinServo.setPosition(urchin_claw_position);
            }
            //**********************************************
            // Lock Hook -- Left Trigger
            //**********************************************
            if (gamepad1.left_trigger != 0) {
                // Keep stepping until servo reaches the max value.
                lock_position += gamepad1.left_trigger * SCALE;

                if (lock_position >= MAX_POS) {
                    lock_position = MAX_POS;
                }
                lockServo1.setPosition(lock_position);
                lockServo2.setPosition(lock_position);
            }
            //**********************************************
            // Release Hook -- Right Trigger
            //**********************************************
            if (gamepad1.right_trigger != 0) {
                // Keep stepping until servo reaches the min value.
                lock_position -= gamepad1.right_trigger * SCALE;

                if (lock_position <= MIN_POS) {
                    lock_position = MIN_POS;
                }
                lockServo1.setPosition(lock_position);
                lockServo2.setPosition(lock_position);
            }
            //**********************************************
            // Rotate Swiper Straws -- X + Left Stick Y
            //**********************************************
            if (gamepad1.left_stick_y != 0 && gamepad1.a) {
                // Keep stepping until servo reaches the min/max value.
                swiper_position += gamepad1.left_stick_y * SCALE;

                if (swiper_position >= MAX_POS) {
                    swiper_position = MAX_POS;
                }
                if (swiper_position <= MIN_POS) {
                    swiper_position = MIN_POS;
                }
                swiperServo.setPosition(swiper_position);
                swiper2.setPosition(swiper_position);
            }

            // Display the commanded servo value

            telemetry.addData(">", "Triangle + L Stick Y = move small Arms","%5.2f", small_rotator_position);
            telemetry.addData(">", "Circle + L Stick Y = rotate urchin", "%5.2f", urchin_rotate_position);
            telemetry.addData(">", "Square + L Stick Y = move clawServo","%5.2f", claw_position);
            telemetry.addData(">", "Left/Right Bumper = Open/Close urchinServo", "%5.2f", urchin_claw_position);
            telemetry.addData(">", "Left/Right Trigger = Lock/Release lockServo", "%5.2f", lock_position);
            telemetry.addData(">", "X + L Stick Y = move swiper servos","%5.2f", swiper_position);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

        }
    }
}
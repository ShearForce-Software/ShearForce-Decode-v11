
/* This code was used during the Freight Frenzy season to test LG's arm servo positions.
It uses a SCALE factor to incrementally move the servo to allow for precise positioning.
Code was developed by Jacob Lemanski and previously list in the menu as "Jacob Servo Test"
*/

package org.firstinspires.ftc.teamcode.LG;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "LG Servo Test")
public class LG_Servo_Test extends LinearOpMode {
    static final double SCALE   = 0.01;     // Joystick scaling for servo increment value
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // Define class members
    Servo liftOne;
    //Servo liftTwo;
    //Servo liftSpin;
    double position1 = 0.0;
    //double position2 = 0.0;// (MAX_POS - MIN_POS) / 2; // Start at halfway position
    //double position3 = 0.0;
    // boolean rampUp = true;

    @Override
    public void runOpMode() {


        // Change the text in quotes to match any servo name on your robot.
        liftOne = hardwareMap.get(Servo.class, "liftOne");
        liftOne.setDirection(Servo.Direction.FORWARD);


        // Wait for the start button
        telemetry.addData(">", "Press Start to move lifeOne Servo with left_stick_y joystick.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.left_stick_y != 0) {
                // Keep stepping until we hit the max/min value.
                position1 += gamepad1.left_stick_y * SCALE;

                if (position1 >= MAX_POS) {
                    position1 = MAX_POS;
                }
                if (position1 <= MIN_POS) {
                    position1 = MIN_POS;
                }
            }

            // Display the current value
            telemetry.addData("liftOne Servo Position", "%5.2f", position1);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
            liftOne.setPosition(position1);
            //sleep(CYCLE_MS);
            idle();
        }
    }
}
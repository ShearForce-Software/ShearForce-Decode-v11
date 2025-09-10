package org.firstinspires.ftc.teamcode.Geronimo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name = "Test_SyncIntakeHangers.java", group = "Test")
@Disabled
public class Test_SyncIntakeHangers extends LinearOpMode {
    Servo intakeHangerLeft;
    Servo intakeHangerRight;
    static final double INCREMENT   = 0.05;
    static final double MAX_POS_RIGHT     =  1.0;
    static final double MIN_POS_RIGHT     =  0.0;
    static final double MAX_POS_LEFT      =  1.0;
    static final double MIN_POS_LEFT      =  0.0;

    @Override
    public void runOpMode() {

        // connect servos- see expansion hub
        //change text in quotes to match name in config file
        intakeHangerLeft = hardwareMap.get(Servo.class, "intakeHangerLeft");
        intakeHangerRight = hardwareMap.get(Servo.class, "intakeHangerRight");

        telemetry.addData(">", "Press Start " );
        telemetry.update();
        waitForStart();
        double positionRight = 0.5;
        double positionLeft = 0.5;

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            if (gamepad2.dpad_up) {
                positionRight = MAX_POS_RIGHT;
                positionLeft = MIN_POS_LEFT;
            }
            else if (gamepad2.dpad_down)
            {
                positionRight = MIN_POS_RIGHT;
                positionLeft = MAX_POS_LEFT;
            }
            else if (gamepad2.dpad_left)
            {
                positionRight = positionRight + INCREMENT;
                positionLeft = positionLeft - INCREMENT;
                if (positionRight > MAX_POS_RIGHT || positionLeft < MIN_POS_LEFT) {
                    positionRight = MAX_POS_RIGHT;
                    positionLeft = MIN_POS_LEFT;
                }
            }
            else if (gamepad2.dpad_right)
            {
                positionRight = positionRight - INCREMENT;
                positionLeft = positionLeft + INCREMENT;
                if (positionRight < MIN_POS_RIGHT || positionLeft > MAX_POS_LEFT)
                {
                    positionRight = MIN_POS_RIGHT;
                    positionLeft = MAX_POS_LEFT;
                }
            }
            intakeHangerLeft.setPosition(positionLeft);
            intakeHangerRight.setPosition(positionRight);
            sleep(50);

            // Display the current value
            telemetry.addData("intakeHangerLeft Position", "%5.2f", positionLeft);
            telemetry.addData("intakeHangerRight Position", "%5.2f", positionRight);
            telemetry.addData(">", "intakeHangerRight is expansion hub port 0." );
            telemetry.addData(">", "intakeHangerLeft is expansion hub port 1." );
            telemetry.addData(">", "intakeHanger is operated by user 2." );
            telemetry.addData(">", "Press dpad_up to increase position to max." );
            telemetry.addData(">", "Press dpad_down to decrease position to min." );
            telemetry.addData(">", "Press dpad_left to increase by increments." );
            telemetry.addData(">", "Press dpad_right to decrease by increments." );
            telemetry.update();

        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}



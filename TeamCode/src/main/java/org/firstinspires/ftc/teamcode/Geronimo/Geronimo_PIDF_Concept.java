package org.firstinspires.ftc.teamcode.Geronimo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * This file contains example PIDF control code inspired by KookyBotz PIDF & Arm Control Video.
 * The Dashboard is used to set rotator arm target position and p, i, d, and f values to calibrate
 * the PIDF controller to smoothly set and hold the two rotator motors on Geromino.
 */


/* Changes to gradle files:
 *
 * build.common.gradle:
 *   repositories {
 *   mavenCentral()      //added for FTC Lib library access (1/1/2025)
 * }
 *
 * build.dependencies.gradle:
 *   repositories {
 *   maven { url ='https://maven.brott.dev'}  //added for FTC Dashboard access (1/1/2025)
 * }
 *
 * build.gradle:
 * dependencies {
 * implementation "org.ftclib.ftclib:core:2.1.1"  //added for FTC Lib library access (1/1/2025)
 * }
 */

@Config
@TeleOp(name="Geronimo: Rotator Arm PIDF Control Concept")
@Disabled
public class Geronimo_PIDF_Concept extends LinearOpMode {

    // proportional, integral, derivative, and feedforward
    public static double p = 0.005, i = 0, d = 0, f = 0.007;

    public static double tolerance = 20.0; //half a degree in ticks
    public static int sendF_to_Controller = 0;
    public static int useTolerance = 0;

    public static int rotator_arm_angle = 0; // target arm angle

    DcMotor leftSlideArmRotatorMotor;
    DcMotor rightSlideArmRotatorMotor;

    @Override
    public void runOpMode() {
        final double arm_gear_ratio = 90.0/20.0;
        final double yellow_jacket_27_ticks = 751.8;    //9.4 ticks for each degree of arm rotation
        final double yellow_jacket_51_ticks = 1425.1;   //17.81 ticks for each degree of arm rotation
        final double ticks_in_degrees = (arm_gear_ratio/360.0) * yellow_jacket_27_ticks;

        double rotator_arm_target = rotator_arm_angle * ticks_in_degrees;

        // Reference: https://docs.ftclib.org/ftclib/features/controllers
        // YouTube reference: https://www.youtube.com/watch?v=E6H6Nqe6qJo
        //Initialize PID Controllers for arm rotator motors.
        PIDController Right_controller = new PIDController(p, i, d);
        PIDController Left_controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app).

        leftSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "leftRotater");
        rightSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "rightRotater");

        leftSlideArmRotatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideArmRotatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Stop and Reset to Zero initially
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            Right_controller.setPID(p,i,d);
             Left_controller.setPID(p,i,d);
          //  Right_controller.setPIDF(p,i,d,f);
         //   Left_controller.setPIDF(p,i,d,f);

            //set tolerance?
            if (useTolerance == 1) {
                Right_controller.setTolerance(tolerance); // sets the error in ticks I think that is tolerated > go back to ticks and degrees, plus or minus the tolerance
                Left_controller.setTolerance(tolerance);
            }
            Left_controller.atSetPoint();
            Right_controller.atSetPoint();

            //TODO- Claire: maybe use tolerance with setpoint code?

          /*   if (Right_controller.atSetPoint() && Left_controller.atSetPoint()){

            }

           */


            if (sendF_to_Controller == 1) {
                Left_controller.setF(f);
                Right_controller.setF(f);
            }

            rotator_arm_target = rotator_arm_angle * ticks_in_degrees;

            //actual arm angle value
            double left_rotator_arm_actual_angle = leftSlideArmRotatorMotor.getCurrentPosition()/ticks_in_degrees;
            double right_rotator_arm_actual_angle = rightSlideArmRotatorMotor.getCurrentPosition()/ticks_in_degrees;

            // Calculate the next PID value
            int left_armPos = leftSlideArmRotatorMotor.getCurrentPosition();
            double left_pid = Left_controller.calculate(left_armPos,rotator_arm_target);

            int right_armPos = rightSlideArmRotatorMotor.getCurrentPosition();
            double right_pid = Right_controller.calculate(right_armPos,rotator_arm_target);

            double left_ff = 0.0;
            double right_ff = 0.0;
            double leftPower = 0.0;
            double rightPower = 0.0;

            if (rotator_arm_angle < 10){
                left_ff = Math.cos(rotator_arm_angle);
                right_ff = Math.cos(rotator_arm_angle);
                leftPower = left_pid + left_ff;
                rightPower = right_pid + right_ff;
            } else {
                left_ff = Math.cos(rotator_arm_angle) * f;
                right_ff = Math.cos(rotator_arm_angle) * f;
                leftPower = left_pid + left_ff;
                rightPower = right_pid + right_ff;
            }

            // Calculate the FeedForward component to adjust the PID by


            // Calculate the motor power (PID + FeedForward) component
         //   double leftPower = left_pid + left_ff;
         //   double rightPower = right_pid + right_ff;

            // Send calculated power to motors
            leftSlideArmRotatorMotor.setPower(leftPower);
            //changed
            rightSlideArmRotatorMotor.setPower(leftPower);


            //Period values or cycle time of current loop
            double Left_period = Left_controller.getPeriod();
            double Right_period = Right_controller.getPeriod();

            // Show arm target, arm positions, and arm power.
            //telemetry for rotator_arm_angle
            telemetry.addData("Rotator arm target(angle)", rotator_arm_angle);
            telemetry.addData("Rotator arm target(ticks)", rotator_arm_target);
            telemetry.addData("pidf: ", "p(%.4f), i(%.4f), d(%.4f), f(%.4f)", p, i, d, f);
            telemetry.addData("tolerance: ", tolerance);
            telemetry.addData("Rotator actual (angle) ", "left (%.2f), right (%.2f)", left_rotator_arm_actual_angle, right_rotator_arm_actual_angle);
            telemetry.addData("Rotator actual (ticks)", "left (%d), right (%d)", left_armPos, right_armPos);
            telemetry.addData("pid calculation values ", "left_pid (%.4f), right_pid (%.4f)", left_pid, right_pid);
            telemetry.addData("ff calculation values", "left_ff (%.4f), right_ff (%.4f)", left_ff, right_ff);
            telemetry.addData("Commanded motor power", "left (%.4f), right (%.4f)", leftPower, rightPower);
            telemetry.addData("Actual motor power", "left (%.4f), right (%.4f)", leftSlideArmRotatorMotor.getPower(), rightSlideArmRotatorMotor.getPower());
            telemetry.addData("period", "left_period (%.4f), right_period (%.4f)", Left_period, Right_period);

            //  telemetry.addData("Rotator motor power", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            //motor get power
        }
    }
}
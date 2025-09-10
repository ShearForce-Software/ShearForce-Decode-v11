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
@TeleOp(name="Geronimo: Slide Extension PIDF Control Concept")
@Disabled
public class Geronimo_Slide_PIDF_Concept extends LinearOpMode {

    // ROTATOR proportional, integral, derivative, and feedforward
    public static double p = 0.004, i = 0, d = 0, f = 0.007;
    public static double Kp = 0.01, Ki = 0, Kd = 0.0002, Kf = 0;


    public static double tolerance = 10.0; //half a degree in ticks
    public static int sendF_to_Controller = 0;
    public static int useTolerance = 0; //if 1 then using tolerance

    public static final int SLIDE_ARM_MIN_POS = 0;
    public static final int SLIDE_ARM_MAX_VERTICAL_POS = 5533; //5918 5300
    public static int slide_ticks = 0; // no conversions for tick extensions
    public static int rotator_arm_angle = 0;

    DcMotor slideLeft;
    DcMotor slideRight;

    DcMotor leftSlideArmRotatorMotor;
    DcMotor rightSlideArmRotatorMotor;

    @Override
    //TODO ASK about Interrupted Exception
    public void runOpMode() {
        final double arm_gear_ratio = 90.0 / 20.0;
       // final double yellow_jacket_27_ticks = 751.8;    //9.4 ticks for each degree of arm rotation
          final double yellow_jacket_51_ticks = 1425.1;   //17.81 ticks for each degree of arm rotation
        final double ticks_in_degrees = (arm_gear_ratio / 360.0) * yellow_jacket_51_ticks;

        double rotator_arm_target = rotator_arm_angle * ticks_in_degrees;

        // Reference: https://docs.ftclib.org/ftclib/features/controllers
        // YouTube reference: https://www.youtube.com/watch?v=E6H6Nqe6qJo
        //Initialize PID Controllers for arm rotator motors.
        //SLIDES
        PIDController RightSlide_controller = new PIDController(Kp, Ki, Kd);
        PIDController LeftSlide_controller = new PIDController(Kp, Ki, Kd);
        //ROTATORS
        PIDController Right_controller = new PIDController(p, i, d);
        PIDController Left_controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app).

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        leftSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "leftRotater");
        rightSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "rightRotater");

        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlideArmRotatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideArmRotatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Stop and Reset to Zero initially
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Stop and Reset to Zero initially
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Use braking to slow the motor down faster?
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            RightSlide_controller.setPID(Kp, Ki, Kd);
            LeftSlide_controller.setPID(Kp, Ki, Kd);
            Right_controller.setPID(p, i, d);
            Left_controller.setPID(p, i, d);

            //set tolerance?
            if (useTolerance == 1) {
                RightSlide_controller.setTolerance(tolerance); // sets the error in ticks I think that is tolerated > go back to ticks and degrees, plus or minus the tolerance
                LeftSlide_controller.setTolerance(tolerance);
            }
            LeftSlide_controller.atSetPoint();
            RightSlide_controller.atSetPoint();

            //TODO- Claire: maybe use tolerance with setpoint code?

            if (sendF_to_Controller == 1) {
                LeftSlide_controller.setF(f);
                RightSlide_controller.setF(f);
            }


            rotator_arm_target = rotator_arm_angle * ticks_in_degrees;

            //actual arm angle value
            double left_rotator_arm_actual_angle = leftSlideArmRotatorMotor.getCurrentPosition() / ticks_in_degrees;
            double right_rotator_arm_actual_angle = rightSlideArmRotatorMotor.getCurrentPosition() / ticks_in_degrees;

            // Calculate the next PID value
            int left_armPos = (leftSlideArmRotatorMotor.getCurrentPosition() + rightSlideArmRotatorMotor.getCurrentPosition())/2;
            double left_pid = Left_controller.calculate(left_armPos, rotator_arm_target);

            int right_armPos = rightSlideArmRotatorMotor.getCurrentPosition();
            double right_pid = Right_controller.calculate(right_armPos, rotator_arm_target);

            // Calculate the FeedForward component to adjust the PID by
            double left_ff = Math.cos(rotator_arm_angle) * f;
            double right_ff = Math.cos(rotator_arm_angle) * f;

            // Calculate the motor power (PID + FeedForward) component
            double leftPower = left_pid + left_ff;
            double rightPower = right_pid + right_ff;

            // Send calculated power to motors
            leftSlideArmRotatorMotor.setPower(leftPower);
            //changed
            rightSlideArmRotatorMotor.setPower(leftPower);

            //slides
            // Make sure slides are never allowed to be commanded to an impossible position that would break the slide motors
            slide_ticks = Math.min(slide_ticks, SLIDE_ARM_MAX_VERTICAL_POS);
            slide_ticks = Math.max(slide_ticks, SLIDE_ARM_MIN_POS);

            double slide_arm_target = slide_ticks;
            //actual arm value
            double left_slide_actual_ticks = slideLeft.getCurrentPosition(); //TODO TELEMETRY
            double right_slide_actual_ticks = slideRight.getCurrentPosition();

            // Calculate the next PID value
            int leftSlide_armPos = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2;
            double leftSlide_pid = LeftSlide_controller.calculate(leftSlide_armPos, slide_arm_target);

            int rightSlide_armPos = slideRight.getCurrentPosition();
            double rightSlide_pid = RightSlide_controller.calculate(rightSlide_armPos, slide_arm_target);

            // Calculate the FeedForward component to adjust the PID by
            double leftSlide_ff = Math.sin(rotator_arm_angle) * Kf;
            double rightSlide_ff = Math.sin(rotator_arm_angle) * Kf;

            // Calculate the motor power (PID + FeedForward) component
            double leftSlidePower = leftSlide_pid + leftSlide_ff;
            double rightSlidePower = rightSlide_pid + rightSlide_ff;

            // Send calculated power to motors
            slideLeft.setPower(leftSlidePower);
            //changed
            slideRight.setPower(leftSlidePower);


            //Period values or cycle time of current loop
            double Left_period = Left_controller.getPeriod();
            double Right_period = Right_controller.getPeriod();

            // Show arm target, arm positions, and arm power.
            //telemetry for rotator_arm_angle
            telemetry.addData("Rotator arm target(angle)", rotator_arm_angle);
            telemetry.addData("Rotator arm target(ticks)", rotator_arm_target);
            telemetry.addData("Rotator pidf: ", "p(%.4f), i(%.4f), d(%.4f), f(%.4f)", p, i, d, f);
            telemetry.addData("Rotator actual (angle) ", "left (%.2f), right (%.2f)", left_rotator_arm_actual_angle, right_rotator_arm_actual_angle);
            telemetry.addData("Rotator actual (ticks)", "left (%d), right (%d)", left_armPos, right_armPos);
            telemetry.addData("Rotator pid calculation values ", "left_pid (%.4f), right_pid (%.4f)", left_pid, right_pid);
            telemetry.addData("Rotator ff calculation values", "left_ff (%.4f), right_ff (%.4f)", left_ff, right_ff);
            telemetry.addData("Commanded Rotator motor power", "left (%.4f), right (%.4f)", leftPower, rightPower);
            telemetry.addData("Actual Rotator motor power", "left (%.4f), right (%.4f)", leftSlideArmRotatorMotor.getPower(), rightSlideArmRotatorMotor.getPower());

            telemetry.addData("Slide arm target(ticks)", slide_ticks);
            telemetry.addData("Slide pidf: ", "p(%.4f), i(%.4f), d(%.4f), f(%.4f)", Kp, Ki, Kd, Kf);
            telemetry.addData("Slide Extension actual (ticks) ", "left (%.2f), right (%.2f)", left_slide_actual_ticks, right_slide_actual_ticks);
            telemetry.addData("Slide left_armPos averaged (ticks)", "left (%d)", left_armPos);
            telemetry.addData("Slide pid calculation values ", "leftSlide_pid (%.4f)", leftSlide_pid);
            telemetry.addData("Slide ff calculation values", "leftSlide_ff (%.4f)", leftSlide_ff);
            telemetry.addData("Commanded Slide motor power", "left (%.4f)", leftSlidePower);
            telemetry.addData("Actual Slide motor power", "left (%.4f), right (%.4f)", slideLeft.getPower(), slideRight.getPower());





            telemetry.addData("tolerance: ", tolerance);
            telemetry.addData("period", "left_period (%.4f), right_period (%.4f)", Left_period, Right_period);

            //  telemetry.addData("Rotator motor power", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            //motor get power


            //  if (Right_controller.atSetPoint() && Left_controller.atSetPoint()){

            //  }


        }
    }



}
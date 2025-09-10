package org.firstinspires.ftc.teamcode.Geronimo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="PIDF_ArmRotator")
@Disabled
//originally just OpMode
public class PIDF_ArmRotator extends LinearOpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    //TODO
    private final double ticks_in_degree = 18 / 1; //18 ticks per 1 degree OR (50:1 motors)

    private DcMotor leftSlideArmRotatorMotor;
    private DcMotor rightSlideArmRotatorMotor;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "leftRotater");
        rightSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "rightRotater");


        waitForStart();
       while (opModeIsActive()) {
            controller.setPID(p, i, d);

            //Only basing it off the left rotator
            int armPosLeft = leftSlideArmRotatorMotor.getCurrentPosition();
            double pid = controller.calculate(armPosLeft, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            //setting power to both rotating motors
            leftSlideArmRotatorMotor.setPower(power);
            rightSlideArmRotatorMotor.setPower(power);

            telemetry.addData("pos", armPosLeft);
            telemetry.addData("target", target);
            telemetry.update();

        }


    }

}


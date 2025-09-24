package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor tester", group = "TEST")
public class intake_test extends LinearOpMode {

    private static final String LEFT_NAME  = "intakeLeft";
    private static final String RIGHT_NAME = "intakeRight";




    private static final double INTAKE_POWER = 0.80;

    private DcMotorEx left, right;
    private DcMotorEx motor3, motor4;
    @Override
    public void runOpMode() {
        left  = hardwareMap.get(DcMotorEx.class, LEFT_NAME);
        right = hardwareMap.get(DcMotorEx.class, RIGHT_NAME);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor3  = hardwareMap.get(DcMotorEx.class, "motor3");
        motor4 = hardwareMap.get(DcMotorEx.class, "motor4");

        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // keep both FORWARD; we’ll use opposite signs for opposite spin
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);

        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Use y sticks on gamepad 1 and 2");
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {

            left.setPower(gamepad1.left_stick_y);
            right.setPower(gamepad1.right_stick_y);// opposite direction

            motor3.setPower(gamepad2.left_stick_y);
            motor4.setPower(gamepad2.right_stick_y);
            // (do nothing if neither is pressed — power stays as-is)

            telemetry.addData("Motor 1",  left.getPower());
            telemetry.addData("Motor 2", right.getPower());

            telemetry.addData("Motor 3",  motor3.getPower());
            telemetry.addData("Motor 4", motor4.getPower());
            telemetry.update();
        }
    }
}

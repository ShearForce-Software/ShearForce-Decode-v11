package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Intake: Start(A) / Stop(B)", group = "TEST")
public class intake_test extends LinearOpMode {

    private static final String LEFT_NAME  = "intakeLeft";
    private static final String RIGHT_NAME = "intakeRight";




    private static final double INTAKE_POWER = 0.80;

    private DcMotorEx left, right;

    @Override
    public void runOpMode() {
        left  = hardwareMap.get(DcMotorEx.class, LEFT_NAME);
        right = hardwareMap.get(DcMotorEx.class, RIGHT_NAME);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // keep both FORWARD; we’ll use opposite signs for opposite spin
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.FORWARD);



        telemetry.addLine("A = INTAKE ON | B = INTAKE OFF");
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {


                left.setPower( INTAKE_POWER);



                right.setPower(-INTAKE_POWER); // opposite direction
            } else if (gamepad1.b) {
                left.setPower(0.0);



                right.setPower(0.0);
            }
            // (do nothing if neither is pressed — power stays as-is)
            telemetry.addData("Left",  left.getPower());



            telemetry.addData("Right", right.getPower());
            telemetry.update();
        }
    }
}

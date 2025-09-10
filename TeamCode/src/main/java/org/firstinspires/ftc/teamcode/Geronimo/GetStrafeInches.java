package org.firstinspires.ftc.teamcode.Geronimo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;

@TeleOp(name = "Get Strafe Inches", group = "Geronimo")
@Disabled
public class GetStrafeInches extends LinearOpMode {

    private Geronimo theRobot;
    private MecanumDrive_Geronimo drive;

    // We’ll store the Limelight strafe offset here
    private double offsetInches = 0.0;

    @Override
    public void runOpMode() {

        // 1) Initialize the robot and Limelight
        theRobot = new Geronimo(false, false, this);
        theRobot.Init(hardwareMap);
        theRobot.InitLimelight(hardwareMap);

        // 2) Initialize the Road Runner Mecanum drive at (0,0,0)
        //    We won't track pose or call setPoseEstimate(), so it's always "starting from zero"
        drive = new MecanumDrive_Geronimo(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Initialized (start pos = 0). Press Play to begin.");
        telemetry.update();
        waitForStart();

        // Variables for edge-detection of dpad presses
        boolean lastDpadUp = false;
        boolean lastDpadRight = false;

        while (opModeIsActive()) {
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadRight = gamepad1.dpad_right;

            //----------------------------------------------------
            //  1) DPAD_UP => GET THE LIMELIGHT OFFSET (ONCE)
            //----------------------------------------------------
            if (currentDpadUp && !lastDpadUp) {
                double [] offsetInches = theRobot.GetStrafeOffsetInInches("block");
                // If offsetInches == 0, it likely means "no target found"
                if (Math.abs(offsetInches[0]) < 0.001 && Math.abs(offsetInches[1])<0.001){
                    telemetry.addLine("No target detected => offset = 0");
                } else {
                    telemetry.addData("Offset (inches in x)", "%.2f", offsetInches[0]);
                    telemetry.addData("Percent of image", "%.2f", theRobot.GivePercentOfTarget());
                }
                telemetry.update();
            }


            //----------------------------------------------------
            //  2) DPAD_RIGHT => STRAFE USING THAT OFFSET
            //----------------------------------------------------
            if (currentDpadRight && !lastDpadRight) {
                // Build a Road Runner action that starts at (0,0,0)
                // and strafes in Y by `offsetInches`.
                Action strafeAction = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToConstantHeading(new Vector2d(0, offsetInches))
                        .build();

                // Run the strafe action (blocking) to move the robot
                Actions.runBlocking(strafeAction);

                // Telemetry
                telemetry.addData("Strafed", "%.2f inches", offsetInches);
                telemetry.update();
            }

            lastDpadUp = currentDpadUp;
            lastDpadRight = currentDpadRight;
        }
    }
}

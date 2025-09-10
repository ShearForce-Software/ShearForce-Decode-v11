package org.firstinspires.ftc.teamcode.Geronimo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

@Autonomous(name = "LimeLightColorAlign", group = "Linear OpMode")
@Disabled
public class LimeLightColorAlign extends LinearOpMode {
    public Limelight3A limelight;
    public Geronimo control;

    // Constants for proportional control
    private static final double KpAim = 0.1; // Proportional control constant for aiming adjustment
    private static final double KpDistance = 0.1; // Proportional control constant for distance adjustment
    private static final double MIN_ADJUST_THRESHOLD = 0.1; // Minimum threshold to stop jittering

    @Override
    public void runOpMode() {
        // Initialize hardware and Limelight
        control = new Geronimo(false, false, this);
        control.Init(hardwareMap);
        control.InitLimelight(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch to color detection pipeline
       // limelight.pipelineSwitch(1); // Ensure pipeline 1 is for color detection
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                double distanceAdjust = -ty * KpDistance;
                double strafeAdjust = -tx * KpAim;

                if (Math.abs(distanceAdjust) > MIN_ADJUST_THRESHOLD || Math.abs(strafeAdjust) > MIN_ADJUST_THRESHOLD) {
                   //control.moveRobot(distanceAdjust, strafeAdjust, 0);
                    telemetry.addData("Tx Adjust", tx);
                    telemetry.addData("Ty Adjust", ty);

                    telemetry.addData("Distance Adjust", distanceAdjust);
                    telemetry.addData("Strafe Adjust", strafeAdjust);
                } else {
                   // control.moveRobot(0, 0, 0);
                    telemetry.addLine("Aligned with target");
                }
                /*
                if (Math.abs(strafeAdjust) > MIN_ADJUST_THRESHOLD) {
                    //control.moveRobot(0, strafeAdjust, 0);
                    telemetry.addData("Distance Adjust", distanceAdjust);
                    telemetry.addData("Strafe Adjust", strafeAdjust);
                } else {
                    // control.moveRobot(0, 0, 0);
                    telemetry.addLine("Aligned with target");
                }
*/
            } else {
                telemetry.addLine("No valid target detected");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}

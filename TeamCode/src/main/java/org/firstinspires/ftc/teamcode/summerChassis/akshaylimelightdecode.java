package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Limelight AprilTag Tracker (minimal)
 * - Outputs left/right offset (in) and forward distance (in) to the detected AprilTag.
 * - Uses Limelight tx/ty plus your camera geometry.
 *
 * Setup:
 * 1) Add the Limelight 3/3A in the RC configuration as "limelight".
 * 2) Ensure your Limelight pipeline detects AprilTags (set in the LL web UI).
 * 3) Tune the constants below for your robot/field.
 */
@TeleOp(name = "LL_AprilTag_Tracker_Minimal", group = "Vision")
@Config
public class akshaylimelightdecode extends LinearOpMode {

    // ====== TUNE FOR YOUR ROBOT/FIELD ======
    // Camera (lens center) height from floor, inches
    public static double CAMERA_HEIGHT_IN = 14.0;

    // Tag center height from floor, inches (set for your FTC prop/tag height)
    public static double TAG_HEIGHT_IN = 18.0;

    // Camera pitch relative to floor, degrees (positive = tilted up)
    public static double CAMERA_PITCH_DEG = 15.0;

    // Limelight pipeline index that detects AprilTags
    public static int APRILTAG_PIPELINE = 0;
    // =======================================

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Direct pipeline switch (no try/catch)
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Limelight AprilTag Tracker (minimal) ready");
        telemetry.addData("Pipeline", APRILTAG_PIPELINE);
        telemetry.update();

        waitForStart();

        final double cameraPitchRad = Math.toRadians(CAMERA_PITCH_DEG);

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double txDeg = result.getTx();                // horizontal angle to tag (deg)
                double tyDeg = result.getTy();                // vertical angle to tag (deg)

                double txRad = Math.toRadians(txDeg);
                double tyRad = Math.toRadians(tyDeg);

                // Forward distance on the floor plane (inches)
                // distanceForward = (tagHeight - cameraHeight) / tan(cameraPitch + ty)
                double deltaH = TAG_HEIGHT_IN - CAMERA_HEIGHT_IN;
                double distanceForwardIn = deltaH / Math.tan(cameraPitchRad + tyRad); // This ignores any left/right offset.

                //tan= opp/adj.  So opp side = change in H(vertical height difference). adja`cent side = forward floor distance.
                //  so we solve for the tan.  forward distance = change in h/tan(angle).

                // Left/Right offset on the floor plane (inches)
                // Positive tx => tag is to the right of camera centerline (LL convention)
                double leftRightOffsetIn = distanceForwardIn * Math.tan(txRad);
                //left/rigt offset = forward distance * tan(x).


                //planar distance combines both the top-down diagonal distance on the carpet. I dunno if this will work or not.
                double planarDistanceIn = Math.hypot(distanceForwardIn, leftRightOffsetIn);

                //formula: planarDistanceIn = sqrt((forward distance)^2 + (left/right offset)^2)  basically pythagorean theorem.

                telemetry.addLine("=== AprilTag ===");
                telemetry.addData("tx (deg)", "%.2f", txDeg);
                telemetry.addData("ty (deg)", "%.2f", tyDeg);
                telemetry.addData("Left/Right offset (in)", "%.2f", leftRightOffsetIn);
                telemetry.addData("Forward distance (in)", "%.2f", distanceForwardIn);
                telemetry.addData("Planar distance (in)", "%.2f", planarDistanceIn);
                telemetry.update();
            } else {
                telemetry.addLine("No AprilTag detected");
                telemetry.update();
            }

            sleep(20);
        }
    }
}

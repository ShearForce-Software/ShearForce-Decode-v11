package org.firstinspires.ftc.teamcode.summerChassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Dual Vision: Limelight + Webcam", group = "Sensor")
public class limelight_webcam extends LinearOpMode {

    // VisionPortal / Webcam objects
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Limelight object
    private Limelight3A limelight;



    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialize Webcam + AprilTag ---
        initAprilTag();  // calling the initAprrilTag;   this is for the webcam.

        // --- Initialize Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Dual Vision Ready. Press PLAY to begin...");
        telemetry.update();
        waitForStart();

        //so the plan is to call the limelight in the opModeIsActive loop itself, and then make method for the webcam telemetry and then call is in the while opModeIsActive there

        while (opModeIsActive()) {
            // --- Webcam Telemetry ---
            telemetryAprilTag();

            // --- Limelight Telemetry ---
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                telemetry.addData("Limelight Tags Detected", fiducialResults.size());

                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    double targetOffsetAngle_Vertical = fr.getTargetYDegrees();
                    double limelightMountAngleDegrees = 0;
                    double limelightLensHeightInches = 5.625;
                    double goalHeightInches = 29.5;

                    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
                    double distanceInches = (goalHeightInches - limelightLensHeightInches) / Math.sin(angleToGoalRadians);

                    telemetry.addLine(String.format("LL Tag ID: %d, Family: %s",
                            fr.getFiducialId(), fr.getFamily()));
                    telemetry.addData("→ Offsets", "X=%.2f°, Y=%.2f°", fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    telemetry.addData("→ Distance", "%.1f in", distanceInches);
                }
            } else {
                telemetry.addLine("Limelight: No target detected");
            }

            telemetry.update();
            sleep(20);
        }

        // Shutdown
        visionPortal.close();
        limelight.stop();




    }

    // Initialize the AprilTag processor and VisionPortal for the webcam.
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();




    }

    // Show telemetry for AprilTag detections from the webcam.
    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("Webcam Tags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("Webcam Tag ID %d (%s)", detection.id, detection.metadata.name));
                telemetry.addData("Range", "%.1f in", detection.ftcPose.range);
                telemetry.addData("Bearing", "%.1f°", detection.ftcPose.bearing);
                telemetry.addData("Elevation", "%.1f°", detection.ftcPose.elevation);

            } else {
                telemetry.addLine(String.format("Webcam Tag ID %d (Unknown)", detection.id));
            }
        }
    }
}

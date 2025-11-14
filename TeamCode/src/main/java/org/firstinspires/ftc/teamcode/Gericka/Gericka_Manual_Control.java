package org.firstinspires.ftc.teamcode.Gericka;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Objects;

@TeleOp(name = "Gericka 1 Manual Control")
//@Disabled
public class Gericka_Manual_Control extends LinearOpMode {
    Gericka_Hardware theRobot;
    boolean rotatorPowerApplied = false;
    boolean slidePowerApplied = false;

    boolean alignBusy = false;

    public int turretTrackingID = 24; // default to Red
    public boolean turretAutoMode = true;
    private boolean dpadDownPrev = false;
    //boolean intakeStarPowerApplied = false;

    float turretRotationAngle = 0.0f;
    final float TURRET_ROTATION_ANGLE_INCREMENT = 1.0f;
    // Scrimmage Meet Ideas:
    // Press intake button, have claw grab the sample
    // Have claw go down when slides extend forward?
    // 1 button to grab specimen off wall (rotate slides, Limelight correction, grab, rotate claw up to get off wall)
    boolean autoLifter = true;
    double shooterSpeedRPM = 0.0;
    final double shooterSpeedRPMIncrement = 50;
    double autoShooterSpeed=0.0;
    boolean autoShooterMode = true;
    boolean useWebcamForDistance = true;
    public void runOpMode() {
        theRobot = new Gericka_Hardware(true, true, this);
        telemetry.setMsTransmissionInterval(11);

        // Use the Blackboard to determine what color we should be
        Object allianceColor = blackboard.get(Gericka_Hardware.ALLIANCE_KEY);
        double defaultHeadingDegrees = 0.0;
        if (Objects.equals(allianceColor, "RED")) {
            theRobot.Init(this.hardwareMap, "RED");
            turretTrackingID = 24;
            theRobot.targetX = -56;
            theRobot.targetY = 56;
            defaultHeadingDegrees = 90.0;
        } else{
            theRobot.Init(this.hardwareMap, "BLUE");
            turretTrackingID = 20;
            theRobot.targetX = -56;
            theRobot.targetY = -56;
            defaultHeadingDegrees = 270.0;
        }

        theRobot.WebcamInit(this.hardwareMap);
        theRobot.SetAutoShooterMode(autoShooterMode);

        // Use the Blackboard to determine the initial x,y and heading that auto finished in, OR use camera view of April tag to set position
        try {
            double xPositionInches = (double) blackboard.get(Gericka_Hardware.FINAL_X_POSITION);
            double yPositionInches = (double) blackboard.get(Gericka_Hardware.FINAL_Y_POSITION);
            double headingDegrees = (double) blackboard.get(Gericka_Hardware.FINAL_HEADING_DEGREES);
            theRobot.SetPinpointPosition(xPositionInches, yPositionInches, headingDegrees);
        } catch (NullPointerException ignored) {
            theRobot.SetPinpointPosition(0.0, 0.0, defaultHeadingDegrees);
            theRobot.pinpoint.update();
            // attempt to use any april tags that are visible to update the pinpoint position
            theRobot.setPinpointPositionFromWebcam();
        }
        theRobot.pinpoint.update();

        theRobot.ShowTelemetry();
        telemetry.update();
        theRobot.InitRoadRunner(hardwareMap);

        waitForStart();
        resetRuntime();


        while (opModeIsActive()) {
            theRobot.EndgameBuzzer();
            /* *************************************************
             *************************************************
             * Driver Controls (gamepad1)
             *************************************************
             *************************************************
             */
            // Drive Controls uses left_stick_y, left_stick_x, and right_stick_x
            theRobot.RunDriveControls();
            // RESERVED COMBOS    options + cross and options + circle

            // Press the triangle button / "y" while facing directly away from the driver to set the IMU correctly for field-centric mode if off
            if (gamepad1.triangleWasPressed()) {
                if (gamepad1.optionsWasPressed()) {
                    theRobot.SetFieldCentricMode(true);
                }
                else {
                    theRobot.imu.resetYaw();
                }
            }
            // robot centric drive mode
            else if (gamepad1.squareWasPressed())
            {
                if (gamepad1.optionsWasPressed()) {
                    theRobot.SetFieldCentricMode(false);
                }
            }



            /* *************************************************
             *************************************************
             * Arm Controls (gamepad2)
             *
             ***************************************              **********
             *************************************************
             */


            // ********   INTAKE MOTOR CONTROLS ***********************
            if (gamepad2.triangleWasPressed() && !gamepad2.optionsWasPressed() && !gamepad2.shareWasPressed()) {
                //Turn intake off
                //theRobot.SetIntakeMotor(true,true);
                theRobot.SetIntakeMotor(false,false);

            }
            else if (gamepad2.circleWasPressed() && !gamepad2.optionsWasPressed() && !gamepad2.shareWasPressed()){
                //Turn intake on (in case of jam?)
                theRobot.SetIntakeMotor(true, true);

            }
            else if (gamepad2.squareWasPressed() && !gamepad2.optionsWasPressed() && !gamepad2.shareWasPressed()){
                //Turn outtake system on
                theRobot.SetIntakeMotor(true,false);
            }
            /*else if(gamepad2.crossWasPressed() && !gamepad2.optionsWasPressed() && !gamepad2.shareWasPressed()){

            }*/
            //TODO - create an auto mode (autoIntake) when sensors available, have a button to turn auto on/off
            //TODO - suggest using (gamepad2.crossWasPressed && !gamepad2.optionsWasPressed() && !gamepad2.shareWasPressed()) for auto intake on/off
            //TODO - create a method in HW class to utilize sensors to auto turn intake on/off -- call here if (autoIntake)



            // ********   LIFTER CONTROLS ***********************
            if (gamepad2.rightBumperWasPressed()) {
                //Set lifter position to up
                theRobot.SetLifterUp();
            }
            else if (gamepad2.rightBumperWasReleased()){
                theRobot.SetLifterDown();
            }
            else if (gamepad1.rightBumperWasPressed()) {
                //Set lifter position to up
                theRobot.SetLifterUp();
            }
            else if (gamepad1.rightBumperWasReleased()){
                theRobot.SetLifterDown();
            }
            else if (gamepad2.leftBumperWasPressed()){
                //Set lifter position to down
                theRobot.SetLifterDown();
            }
            else if (gamepad2.left_trigger > 0.2){
                //Set lifter position to middle
                theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);
            }
            else if (gamepad2.right_trigger > 0.2){
                if (gamepad2.shareWasPressed()) {
                    if (autoLifter) {
                        autoLifter = false;
                    }
                    else {
                        autoLifter = true;
                    }
                }
            }
            // Run the Auto Lift to Midway position (if enabled)
            theRobot.LifterAuto(autoLifter);

            // ********   TURRET CONTROLS ***********************
            if (gamepad2.circleWasPressed() ){
                if (gamepad2.shareWasPressed()) {
                    //Turn Turret clockwise
                    turretRotationAngle += TURRET_ROTATION_ANGLE_INCREMENT;
                    theRobot.SetTurretRotationAngle(turretRotationAngle);
                    turretAutoMode = false;
                }
            }
            else if (gamepad2.squareWasPressed() ){
                if (gamepad2.shareWasPressed()) {
                    //Turn Turret counterclockwise
                    turretRotationAngle -= TURRET_ROTATION_ANGLE_INCREMENT;
                    theRobot.SetTurretRotationAngle(turretRotationAngle);
                    turretAutoMode = false;
                }
            }
            // switch to tracking red target
            if (gamepad2.triangleWasPressed() ){
                if (gamepad2.shareWasPressed()) {
                    if (turretTrackingID == 20) {
                        // switch turret tracking to RED target
                        turretTrackingID = 24;
                        turretAutoMode = true;
                    } else {
                        // switch turret tracking to BLUE target
                        turretTrackingID = 20;
                        turretAutoMode = true;
                    }
                }
            }

            // toggling turret goal auto centering/tracking on/off
            if ( gamepad2.crossWasPressed() && !gamepad2.optionsWasPressed() ) {
                if (gamepad2.shareWasPressed()) {
                    turretAutoMode = !turretAutoMode;
                }
            }

            if (turretAutoMode) {
                theRobot.adjustTurretToTargetAprilTag(turretTrackingID);
            }


            // ********   SHOOTER MOTOR CONTROLS ***********************
            if (gamepad2.dpadUpWasPressed()){
                autoShooterMode = false;
                theRobot.SetAutoShooterMode(autoShooterMode);
                shooterSpeedRPM = 0.0;
                theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
            }
            else if (gamepad2.dpadLeftWasPressed()){
                if (gamepad2.optionsWasPressed()){
                    autoShooterMode = false;
                    theRobot.SetAutoShooterMode(autoShooterMode);

                    shooterSpeedRPM = 2100; //4500rpm was about the value observed when the Motor was commanded to 100%.
                    theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
                }
                else {
                    autoShooterMode = false;
                    theRobot.SetAutoShooterMode(autoShooterMode);
                    shooterSpeedRPM -= shooterSpeedRPMIncrement;
                    theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
                }
            }
            else if (gamepad2.dpadRightWasPressed()){
                if (gamepad2.optionsWasPressed()){
                    autoShooterMode = false;
                    theRobot.SetAutoShooterMode(autoShooterMode);
                    shooterSpeedRPM = 2950; //1950rpm was about the value observed when the Motor was commanded to 50%.
                    theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
                }
                else {
                    autoShooterMode = false;
                    theRobot.SetAutoShooterMode(autoShooterMode);
                    shooterSpeedRPM += shooterSpeedRPMIncrement;
                    theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
                }
            }
            else if (gamepad2.dpadDownWasPressed()) {
                if (gamepad2.optionsWasPressed()) {
                    autoShooterMode = false;
                    theRobot.SetAutoShooterMode(autoShooterMode);
                    shooterSpeedRPM = 3500; //3200rpm was about the value observed when the Motor was commanded to 75%.
                    theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
                }
                else {
                    autoShooterMode = !autoShooterMode;
                    theRobot.SetAutoShooterMode(autoShooterMode);
                }
            }
            if (autoShooterMode) {
                if (useWebcamForDistance) {
                    theRobot.ShooterRPMFromWebCam(theRobot.currentAprilTargetId);
                } else {
                    theRobot.ShooterRPMFromPinpoint();
                }
            }

            /*
            24 inch - 2100rpm
            30 inch - 2200rpm
            36 inch - 2300rpm
            42 inch - 2350rpm
            48 inch - 2350rpm
            54 inch - 2400rpm
            60 inch - 2650rpm
            66 inch - 2750rpm
            72 inch - 2800rpm
            78 inch - 2950rpm
            far launch zone (120 inch) - 3500rpm


            24, 80(tip of large triangle), 120 inch for tip of small triangle

             */


            theRobot.ShowTelemetry();
        } // end while (opModeIsActive())

    }
    }

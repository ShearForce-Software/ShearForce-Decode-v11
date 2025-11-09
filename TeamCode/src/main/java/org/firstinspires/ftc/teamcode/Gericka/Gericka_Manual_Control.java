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
    public static final String ALLIANCE_KEY = "Alliance";

    int turretTrackingID = 24; // default to Red
    public boolean turretAutoMode = true;
    private boolean dpadDownPrev = false;
    //boolean intakeStarPowerApplied = false;

    float turretRotationAngle = 0.0f;
    final float TURRET_ROTATION_ANGLE_INCREMENT = 1.0f;
    // Scrimmage Meet Ideas:
    // Press intake button, have claw grab the sample
    // Have claw go down when slides extend forward?
    // 1 button to grab specimen off wall (rotate slides, Limelight correction, grab, rotate claw up to get off wall)
    double shooterSpeedRPM = 0.0;
    final double shooterSpeedRPMIncrement = 50;

    public void runOpMode() {
        theRobot = new Gericka_Hardware(true, true, this);
        telemetry.setMsTransmissionInterval(11);

        // Use the Blackboard to determine what color we should be
        Object allianceColor = blackboard.get(ALLIANCE_KEY);
        if (Objects.equals(allianceColor, "RED")) {
            theRobot.Init(this.hardwareMap, "RED");
            turretTrackingID = 24;
        } else{
            theRobot.Init(this.hardwareMap, "BLUE");
            turretTrackingID = 20;
        }

        theRobot.WebcamInit(this.hardwareMap);

        // Use the Blackboard to determine the initial x,y and heading that auto finished in, OR use camera view of April tag to set position
        //TODO use the blackboard function to set x,y, and heading at end of Auto, then reinitialize the pinpoint with that position OR USE camera and APRIL TAG
        double xPositionInches = 0.0;
        double yPositionInches = 0.0;
        double headingDegrees = 0.0;
        theRobot.SetPinpointPosition(xPositionInches, yPositionInches, headingDegrees);

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
            if (gamepad1.triangle && !gamepad1.options) {
                theRobot.imu.resetYaw();
            }
            // field centric drive mode (default at startup)
            else if (gamepad1.triangle && gamepad1.options)
            {
                theRobot.SetFieldCentricMode(true);
            }
            // robot centric drive mode
            else if (gamepad1.square && gamepad1.options)
            {
                theRobot.SetFieldCentricMode(false);
            }



            /* *************************************************
             *************************************************
             * Arm Controls (gamepad2)
             *
             *************************************************
             *************************************************
             */


            // ********   INTAKE MOTOR CONTROLS ***********************
            if (gamepad2.triangleWasPressed() && !gamepad2.optionsWasPressed() && !gamepad2.share) {
                //Turn intake on
                theRobot.SetIntakeMotor(true,true);

            }
            else if (gamepad2.circleWasPressed() && !gamepad2.optionsWasPressed() && !gamepad2.share){
                //Turn outake on (in case of jam?)
                theRobot.SetIntakeMotor(true, false);
            }
            else if (gamepad2.squareWasPressed() && !gamepad2.optionsWasPressed() && !gamepad2.share){
                //Turn intake system off
                theRobot.SetIntakeMotor(false,false);
            }
            //TODO - create an auto mode (autoIntake) when sensors available, have a button to turn auto on/off
            //TODO - suggest using (gamepad2.crossWasPressed && !gamepad2.options && !gamepad2.share) for auto intake on/off
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
            //TODO Add method HW class to utilize sensors to auto lift ball to MID position, call here if autoLift mode is true
            //TODO add combo button to turn auto lift on/off -- suggest (gamepad2.share && gamepad2.right_trigger > 0.2)



            // ********   TURRET CONTROLS ***********************
            if (gamepad2.circleWasPressed() && gamepad2.share){
                //Turn Turret clockwise
                turretRotationAngle += TURRET_ROTATION_ANGLE_INCREMENT;
                theRobot.SetTurretRotationAngle(turretRotationAngle);
                turretAutoMode = false;
            }
            else if (gamepad2.squareWasPressed() && gamepad2.share){
               //Turn Turret counterclockwise
                turretRotationAngle -= TURRET_ROTATION_ANGLE_INCREMENT;
                theRobot.SetTurretRotationAngle(turretRotationAngle);
                turretAutoMode = false;
            }
            // switch to tracking red target
            if (gamepad2.triangleWasPressed() && gamepad2.share){
                if ( turretTrackingID == 20) {
                    // switch turret tracking to RED target
                    turretTrackingID = 24;
                    turretAutoMode = true;
                }
                else {
                    // switch turret tracking to BLUE target
                    turretTrackingID = 20;
                    turretAutoMode = true;
                }
            }

            // toggling turret goal auto centering/tracking on/off
            if (gamepad2.share && gamepad2.crossWasPressed() && !gamepad2.optionsWasPressed() ) {
                turretAutoMode = !turretAutoMode;
            }

            if (turretAutoMode) {
                theRobot.adjustTurretToTarget(turretTrackingID);
            }



            // ********   SHOOTER MOTOR CONTROLS ***********************
            if (gamepad2.dpadUpWasPressed()){
                shooterSpeedRPM = 0.0;
                theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
            }
            else if (gamepad2.dpadLeftWasPressed() && !gamepad2.options){
                shooterSpeedRPM -= shooterSpeedRPMIncrement;
                theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
            }
            else if (gamepad2.dpadRightWasPressed() && !gamepad2.options){
                shooterSpeedRPM += shooterSpeedRPMIncrement;
                theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
            }
             else if (gamepad2.dpadLeftWasPressed() && gamepad2.options){
                shooterSpeedRPM = 2000; //4500rpm was about the value observed when the Motor was commanded to 100%.
                theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
            }
            else if (gamepad2.dpadRightWasPressed() && gamepad2.options){
                shooterSpeedRPM = 3200; //1950rpm was about the value observed when the Motor was commanded to 50%.
                theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
            }
            else if (gamepad2.dpadDownWasPressed() && gamepad2.options){
                shooterSpeedRPM = 4500; //3200rpm was about the value observed when the Motor was commanded to 75%.
                theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
            }
            //TODO - add an auto rpm set mode (autoShooterSpeed), turn on/off via button, suggest (gamepad2.dpadDownWasPressed() && !gamepad2.options)
            //TODO - add a method in HW class to auto set RPM based on webcam distance, call here if (autoShooterSpeed == true)
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
            78 inch - 2900rpm
            far launch zone (120 inch) - 3500rpm
             */



            theRobot.ShowTelemetry();
        } // end while (opModeIsActive())

    }
}

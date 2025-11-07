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

    public void runOpMode() {
        theRobot = new Gericka_Hardware(true, true, this);
        telemetry.setMsTransmissionInterval(11);
        Object allianceColor = blackboard.get(ALLIANCE_KEY);
        if (Objects.equals(allianceColor, "RED")) {
            theRobot.Init(this.hardwareMap, "RED");
            turretTrackingID = 24;
        } else{
            theRobot.Init(this.hardwareMap, "BLUE");
            turretTrackingID = 20;
        }
        theRobot.WebcamInit(this.hardwareMap);
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
            if (gamepad2.triangleWasPressed()) {
                //Turn intake on
                theRobot.SetIntakeMotor(true,true);

            }
            else if (gamepad2.circleWasPressed() && !gamepad2.optionsWasPressed()){
                //Turn outake on
                theRobot.SetIntakeMotor(true, false);
            }
            else if (gamepad2.squareWasPressed()){
                //Turn intake system off
                theRobot.SetIntakeMotor(false,false);
            }



            // ********   LIFTER CONTROLS ***********************
            if (gamepad2.rightBumperWasPressed()) {
                //Set lifter position to up
                theRobot.SetLifterUp();
            }
            else if (gamepad2.rightBumperWasReleased()){
                theRobot.SetLifterDown();
            }
            else if (gamepad2.leftBumperWasPressed()){
                //Set lifter position to down
                theRobot.SetLifterPosition(theRobot.LIFTER_DOWN_POSITION);
            }
            else if (gamepad2.left_trigger > 0.2){
                //Set lifter position to middle
                theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);
            }



            // ********   TURRET CONTROLS ***********************
            if (gamepad1.dpadRightWasPressed()){
                //Turn Turret clockwise
                turretRotationAngle += TURRET_ROTATION_ANGLE_INCREMENT;
                theRobot.SetTurretRotationAngle(turretRotationAngle);
            }
            else if (gamepad1.dpadLeftWasPressed()){
               //Turn Turret counterclockwise
                turretRotationAngle -= TURRET_ROTATION_ANGLE_INCREMENT;
                theRobot.SetTurretRotationAngle(turretRotationAngle);
            }
            // switch to tracking red target
            if (gamepad1.dpad_up){
                turretTrackingID = 24;
                //red
            }
            // switch to tracking blue target
            else if (gamepad1.dpad_up && gamepad1.options){
                turretTrackingID = 20;
                //blue
            }
            // toggling turret goal auto centering/tracking on/off
            //TODO - need an auto tracking mode that can be turned on/off
            if (gamepad2.share && gamepad2.dpadLeftWasPressed()) {
                turretAutoMode = false;
            }
            else if (gamepad2.share && gamepad2.dpadRightWasPressed()){
                turretAutoMode = true;
            }

            //TODO - need a different button or combo to do this manual adjustment if you really want it, share is common combo button
            if (gamepad2.share && gamepad2.dpadDownWasPressed()) {
                theRobot.adjustTurretToTarget(turretTrackingID);
            }
            //TODO - WHAT does this do?  Looks like it tells it to do nothing (SetTurret to what the turret is currently set to)
            //else if (gamepad2.share && gamepad2.options) {
                //theRobot.SetTurretRotationAngle(theRobot.getCurrentTurretAngle());
            //}
            //TODO -- should have simple logic here to auto track all the time, something like:  if (turretAutoMode) { theRobot.adjustTurretToTarget(turretTrackingID); }
            if (turretAutoMode) {
                theRobot.adjustTurretToTarget(turretTrackingID);
            }


            // ********   SHOOTER MOTOR CONTROLS ***********************
            //TODO - should create a local variable like double shooterSpeedRPM, and set that value, then just always call SetShooterMotorToSpecificRPM(shooterSpeedRPM), so that can have increment/decrement commands too
            //TODO - add command combos that can be used to command the shooter speed to change by set increments like 50 RPMs or something
            //TODO - Need to change to commanding RPMs -- test how well it holds the RPM value, suspect pretty well
            if (gamepad2.dpadUpWasPressed()){
                //Set shooter wheel speed to 0
                theRobot.SetShooterSpeed(0.0);
            }
            else if (gamepad2.dpadLeftWasPressed()){
                //Set shooter speed to 25
                theRobot.SetShooterSpeed(0.5);
            }
            else if (gamepad2.dpadDownWasPressed()){
                //Set shooter speed to 50
                theRobot.SetShooterSpeed(0.75);
            }
            else if (gamepad2.dpadRightWasPressed()){
                //Set shooter speed to 100
                theRobot.SetShooterSpeed(1.0);
            }



            theRobot.ShowTelemetry();
        } // end while (opModeIsActive())

    }
}

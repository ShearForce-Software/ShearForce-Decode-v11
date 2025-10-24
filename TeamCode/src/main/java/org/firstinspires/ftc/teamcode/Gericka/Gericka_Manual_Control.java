package org.firstinspires.ftc.teamcode.Gericka;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Gericka.Gericka_Hardware;

@TeleOp(name = "Gericka 1 Manual Control")
//@Disabled
public class Gericka_Manual_Control extends LinearOpMode {
    Gericka_Hardware theRobot;
    boolean rotatorPowerApplied = false;
    boolean slidePowerApplied = false;

    boolean alignBusy = false;

    private boolean dpadDownPrev = false;
    //boolean intakeStarPowerApplied = false;

    float turretRotationAngle = 0.0f;

    final float TURRET_ROTATION_ANGLE_INCREMENT = 5.0f;
    // Scrimmage Meet Ideas:
    // Press intake button, have claw grab the sample
    // Have claw go down when slides extend forward?
    // 1 button to grab specimen off wall (rotate slides, Limelight correction, grab, rotate claw up to get off wall)

    public void runOpMode() {
        theRobot = new Gericka_Hardware(true, true, this);
        telemetry.setMsTransmissionInterval(11);

        theRobot.Init(this.hardwareMap);
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

            // 3) then all your other gamepad1 / gamepad2 commands
            if (gamepad1.triangle && !gamepad1.options) {
                theRobot.imu.resetYaw();
            } else if (gamepad1.triangle && gamepad1.options) {
                theRobot.SetFieldCentricMode(true);

            } else if (gamepad1.square && gamepad1.options) {
                theRobot.SetFieldCentricMode(false);
            }

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

            if (gamepad2.triangleWasPressed()) {
                //Turn intake on
                theRobot.SetIntakeMotor(true,true);

            }
            else if (gamepad2.circleWasPressed()){
                //Turn outake on
                theRobot.SetIntakeMotor(true, false);
            }
            else if (gamepad2.squareWasPressed()){
                //Turn intake system off
                theRobot.SetIntakeMotor(false,false);
            }

            if (gamepad2.rightBumperWasPressed()){
                //Set lifter position to up
                theRobot.SetLifterPosition(theRobot.LIFTER_UP_POSITION);
            }
            else if (gamepad2.leftBumperWasPressed()){
                //Set lifter position to down
                theRobot.SetLifterPosition(theRobot.LIFTER_DOWN_POSITION);
            }

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

            if (gamepad2.dpadUpWasPressed()){
                //Set shooter wheel speed to 0
                theRobot.SetShooterSpeed(0.0f);
            }
            else if (gamepad2.dpadLeftWasPressed()){
                //Set shooter speed to 25
                theRobot.SetShooterSpeed(0.25f);
            }
            else if (gamepad2.dpadDownWasPressed()){
                //Set shooter speed to 50
                theRobot.SetShooterSpeed(0.5f);
            }
            else if (gamepad2.dpadRightWasPressed()){
                //Set shooter speed to 100
                theRobot.SetShooterSpeed(1.0f);
            }

            theRobot.ShowTelemetry();
        } // end while (opModeIsActive())

    }
}

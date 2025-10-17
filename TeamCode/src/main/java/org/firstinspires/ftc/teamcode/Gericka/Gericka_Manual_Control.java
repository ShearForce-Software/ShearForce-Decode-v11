package org.firstinspires.ftc.teamcode.Gericka;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


            theRobot.ShowTelemetry();
        } // end while (opModeIsActive())

    }
}

package org.firstinspires.ftc.teamcode.Gericka;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Gericka.Gericka_Hardware;
import org.firstinspires.ftc.teamcode.Gericka.Gericka_MecanumDrive;
import org.firstinspires.ftc.teamcode.Geronimo.FourHighSpecimensAutoRoute;

// auto select manual opMode next
@Autonomous(name="Red Far Auto", preselectTeleOp ="Gericka 1 Manual Control")

public class Red_Far_Auto extends LinearOpMode {
    Gericka_Hardware control = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;

    // Trajectories

    Action DriveToSecondMark;
    Action DriveToShootingPosition;
    Action DriveSecondMarkToSmallTriangle;
    Action DriveToFirstMark;
    Action DriveFirstMarkToSmallTriangle;
    Action DriveOutofLaunchZone;

    public void runOpMode() {
        startPose = new Pose2d(60, 12, Math.toRadians(90));
        /* Initialize the Robot */
        control.Init(hardwareMap, "RED");

        // initialize roadrunner
        drive = new Gericka_MecanumDrive(hardwareMap, startPose);

        // initialize the webcam
        control.WebcamInit(this.hardwareMap);

        sleep(500); // sleep at least 1/4 second to allow pinpoint to calibrate itself
        // finish initializing the pinpoint
        control.SetPinpointPosition(60, 12, 90);

        blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "RED");

        // set lifter half up (so can get 3 ball loaded in robot)
        control.SetLifterPosition(control.LIFTER_MID_POSITION);

        //control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        // *** Route aligns with Decode_MeepMeep_Blue_ID22_Small_Triangle ***
        DriveToShootingPosition = drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48,12))
                .build();

        DriveToSecondMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(11.5, 30))
                .strafeToConstantHeading(new Vector2d(11.5, 57))
                .build();

        DriveSecondMarkToSmallTriangle = drive.actionBuilder(new Pose2d(11.5, 57, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(20, 12))
                .strafeToConstantHeading(new Vector2d(48, 12))
                .build();

        DriveToFirstMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(34.75, 30))
                .strafeToConstantHeading(new Vector2d(34.75, 57))
                .build();

        DriveFirstMarkToSmallTriangle = drive.actionBuilder(new Pose2d(34.75, 57, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12))
                .build();

        DriveOutofLaunchZone = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(20, 12))
                .build();

        boolean autoLifter = true;

        // ***************************************************
        // ****  Secondary Thread to run all the time ********
        // ***************************************************
        Thread SecondaryThread = new Thread(() -> {
            while (!isStopRequested() && getRuntime() < 30) {
                control.ShowTelemetry();
                //control.ShowPinpointTelemetry();

                if (isStarted()) {
                    control.LifterAuto(autoLifter);
                }
                telemetry.update();

                sleep(20);
            }
        });
        SecondaryThread.start();

        // turn turret to face the obelisk
        double turretTargetAngle = -91.0;
        control.SetTurretRotationAngle(turretTargetAngle);
        sleep(1000);
        // turn off turret power so doesn't twitch
        control.turretMotor.setPower(0);

        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        while (!isStarted() && !isStopRequested()) {
            telemetry.update();
        }

        // ********* STARTED ********************************
        resetRuntime();
        Gericka_Hardware.autoTimeLeft = 0.0;

        // turn on intake to suck in any stuck balls
        control.SetIntakeMotor(true,true);

        // spin up shooter wheel to max
        //control.SetShooterSpeed(1.0);
        // set shooter speed to small triangle speed, can just leave at this speed the whole time
        double shooterSpeedRPM = 3500;
        control.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // drive to the start triangle
        Actions.runBlocking(DriveToShootingPosition);

        //TODO Read the obelisk apriltag, display result in telemetry (we don't really need it yet, but should start assessing our ability to get it)

        // Turn Turret towards target, can leave turret there the whole time
        turretTargetAngle = -115.0;
        control.SetTurretRotationAngle(turretTargetAngle);

        // sleep some time to allow shooter wheel to spin up
        while (control.CalculateMotorRPM(control.shooterMotorLeft.getVelocity(), control.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 100)) {
            sleep(20);
        }

        /* **** SHOOT BALL #1 **** */
        control.SetLifterUp();  // Shot 1
        sleep(500); //TODO need to find the smallest sleep time to lift and get shot
        control.SetLifterDown();
        sleep(500); //TODO need to find the smallest sleep time to drop and get another ball ready

        // can stop the intake after the first shot to save power
        control.SetIntakeMotor(false,true);

        // sleep some time to allow shooter wheel to spin back up if needed
        while (control.CalculateMotorRPM(control.shooterMotorLeft.getVelocity(), control.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 100)) {
            sleep(20);
        }

        /* **** SHOOT BALL #2 **** */
        control.SetLifterUp();  // Shot 2
        sleep(500);
        control.SetLifterDown();
        sleep(500);

        // sleep some time to allow shooter wheel to spin back up if needed
        while (control.CalculateMotorRPM(control.shooterMotorLeft.getVelocity(), control.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 100)) {
            sleep(20);
        }

        /* **** SHOOT BALL #3 **** */
        control.SetLifterUp();  // Shot 3
        sleep(500);
        control.SetLifterDown();

        //shooterSpeedRPM = 3500;
        //control.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        // Drive to the middle line and turn the intake on
                        new ParallelAction(DriveToSecondMark, setIntakeOn()),
                        new SleepAction(1.0), // tiny sleep to finish ingesting balls, not sure how much is really needed
                         // Return to launch zone and turn intake off
                        new ParallelAction(DriveSecondMarkToSmallTriangle, setIntakeOff()),

                        // Shoot 3 balls
                        new SequentialAction(
                            new SetLifterUp(),
                            new SleepAction(1),
                            new SetLifterDown(),
                            new SleepAction(1),
                            new SetLifterUp(),
                            new SleepAction(1),
                            new SetLifterDown(),
                            new SleepAction(1),
                            new SetLifterUp(),
                            new SleepAction(1),

                            // Drive to closest line and turn intake on and lower lifter
                            new ParallelAction(DriveToFirstMark, setIntakeOn(), new SetLifterDown())),
                            new SleepAction(1.0) // tiny sleep to finish ingesting balls, not sure how much is really needed
                             //TODO add parallel action that turns intake off
                /*            new ParallelAction(DriveFirstMarkToSmallTriangle, setIntakeOff()),
                            //new SleepAction(2), //TODO shoot 3 here

                        new SequentialAction(
                                new SetLifterUp(),
                                new SleepAction(1),
                                new SetLifterDown(),
                                new SleepAction(1),
                                new SetLifterUp(),
                                new SleepAction(1),
                                new SetLifterDown(),
                                new SleepAction(1),
                                new SetLifterUp(),
                                new SleepAction(1)),
                        DriveOutofLaunchZone));

                 */));

        drive.updatePoseEstimate();

        // store final exact position in blackboard, so can initialize manual pinpoint with that position
        control.pinpoint.update();
        blackboard.put(Gericka_Hardware.FINAL_X_POSITION, control.pinpoint.getPosX(DistanceUnit.INCH));
        blackboard.put(Gericka_Hardware.FINAL_Y_POSITION, control.pinpoint.getPosY(DistanceUnit.INCH));
        blackboard.put(Gericka_Hardware.FINAL_HEADING_DEGREES, control.pinpoint.getHeading(AngleUnit.DEGREES));

        // turn off shooter wheel
        shooterSpeedRPM = 0.0; //3200rpm was about the value observed when the Motor was commanded to 75%.
        control.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // return turret to zero position
        turretTargetAngle = 0.0;
        control.SetTurretRotationAngle(turretTargetAngle);

        Gericka_Hardware.autoTimeLeft = 30 - getRuntime();
        telemetry.addData("Time left", Gericka_Hardware.autoTimeLeft);
        telemetry.update();


    }

    public Action setIntakeOn() {
        return new setIntakeOn();
    }

    public class setIntakeOn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            control.SetIntakeMotor(true, true);
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
            }
        }

        public Action setIntakeOff() {
            return new setIntakeOff();
        }

        public class setIntakeOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }
                control.SetIntakeMotor(false, false);
                packet.put("lock purple pixel", 0);
                return false;  // returning true means not done, and will be called again.  False means action is completely done
            }
        }

    public class SetLifterUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            control.SetLifterUp();
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

    public class SetLifterDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            control.SetLifterDown();
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

}

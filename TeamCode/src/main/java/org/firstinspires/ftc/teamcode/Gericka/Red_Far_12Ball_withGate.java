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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// auto select manual opMode next
@Autonomous(name = "RED Far Auto 12 GATE balls", preselectTeleOp = "Gericka 1 Manual Control")
public class

Red_Far_12Ball_withGate extends LinearOpMode {

    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;

    // Trajectories
    Action DriveToShootingPosition;
    Action DriveToSecondMark;
    Action DriveThirdMarkToBigTriangle;
    Action SecondMarkToLock;
    Action LockToBigTriangle;
    Action DriveBigTriangleToThirdMark;
    Action DriveBigTriangleToFirstMark;
    Action DriveFirstMarkToLock;
    Action DriveFirstMarkToShootingPosition;
    Action DriveShootingPositionToGateLock;

    // Constraints
    VelConstraint fastVel = new TranslationalVelConstraint(90);
    AccelConstraint fastAccel = new ProfileAccelConstraint(-65, 65);

    VelConstraint intakeVel = new TranslationalVelConstraint(60);
    AccelConstraint intakeAccel = new ProfileAccelConstraint(-30, 40);

    VelConstraint loopVel = new TranslationalVelConstraint(40);
    AccelConstraint loopAccel = new ProfileAccelConstraint(-25, 25);

    VelConstraint specialVel = new TranslationalVelConstraint(40);
    AccelConstraint specialAccel = new ProfileAccelConstraint(-15, 15);

    int lifterUpSleepTime = 300;
    int lifterDownSleepTime = 400;

    @Override
    public void runOpMode() {
        startPose = new Pose2d(60, 12, Math.toRadians(90));
        /* Initialize the Robot */
        theRobot.Init(hardwareMap, "RED");

        // initialize roadrunner
        drive = new Gericka_MecanumDrive(hardwareMap, startPose);
        theRobot.InitRoadRunner(drive);


        // initialize the webcam
        theRobot.WebcamInit(this.hardwareMap);

        // Turn turret toward the obelisk BEFORE scanning (useful if the camera is turret-mounted)
        double turretTargetAngle = -115;
        theRobot.SetTurretRotationAngle(turretTargetAngle);
        theRobot.SetLaunchRampPosition(1.0);

        sleep(3000); // allow turret to reach position
        // turn off turret power so it doesn't twitch during init
        theRobot.TurnOffTurret();

        // finish initializing pinpoint / roadrunner initial position
        sleep(500);
        theRobot.SetRoadrunnerInitialPosition(60, 12, 90);

        blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "RED");

        // set lifter half up (so can get 3 balls loaded in robot)
        theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        //Shoot 3 preloaded balls
        DriveToShootingPosition = drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                .build();

        //DriveToSecondMarkToCollectBallsandgetclosertogatelock
        DriveToSecondMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(11.5, 30), Math.toRadians(90), fastVel, fastAccel)
                .splineToConstantHeading(new Vector2d(11.5, 60), Math.toRadians(90), intakeVel, intakeAccel)
                .build();

        SecondMarkToLock = drive.actionBuilder(new Pose2d(11.5, 60, Math.toRadians(90)))   // FIX
                // U-loop to bump/open the gate near the top wall, then come back down to the lock
                // (tweak these waypoints to match your exact gate location)
                .splineToConstantHeading(new Vector2d(5, 40),  Math.toRadians(180), loopVel, loopAccel)  // up-left
                .splineToConstantHeading(new Vector2d(0, 52),  Math.toRadians(90), loopVel, loopAccel)  // up-left
                .build();

        LockToBigTriangle = drive.actionBuilder(new Pose2d(0, 50, Math.toRadians(90)))   // FIX
                .splineToConstantHeading(new Vector2d(0, 20),  Math.toRadians(270), intakeVel, intakeAccel)  // up-left
                .splineToConstantHeading(new Vector2d(-11.5, 21),  Math.toRadians(90),specialVel, specialAccel)  // up-left
                .build();

        DriveBigTriangleToThirdMark = drive.actionBuilder(new Pose2d(-11.5, 21, Math.toRadians(90)))   // FIX
                .splineToConstantHeading(new Vector2d(-15, 31), Math.toRadians(90), loopVel, loopAccel)
                .strafeToConstantHeading(new Vector2d(-15, 60), loopVel, loopAccel)
                .build();

        DriveThirdMarkToBigTriangle = drive.actionBuilder(new Pose2d(-15, 60, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-11.5, 25), fastVel, fastAccel)
                .build();

        DriveBigTriangleToFirstMark =  drive.actionBuilder(new Pose2d(-11.5, 25, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(34.75, 30), fastVel, fastAccel)
                //.splineToConstantHeading(new Vector2d(34.75, 60), Math.toRadians(90), intakeVel, intakeAccel)
                .strafeToConstantHeading(new Vector2d(34.75, 60), intakeVel, intakeAccel)
                .build();

        DriveFirstMarkToShootingPosition =  drive.actionBuilder(new Pose2d(34.74, 60, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                .build();

        DriveFirstMarkToLock =  drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, 40), fastVel, fastAccel)
                .build();

        DriveShootingPositionToGateLock =  drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, 40), fastVel, fastAccel)
                .build();


        theRobot.SetAutoLifterMode(true);
        theRobot.SetShooterPIDF_Enabled(true);
        Gericka_Hardware.shooterF = theRobot.PIDF_F_SMALL_TRIANGLE;

        // ***************************************************
        // ****  Secondary Thread to run all the time ********
        // ***************************************************
        Thread SecondaryThread = new Thread(() -> {
            while (!isStopRequested() && getRuntime() < 30) {
                theRobot.ShowTelemetry();
                theRobot.SetIndicatorLights();

                if (isStarted()) {
                    theRobot.RunAutoLifter();
                    theRobot.SetShooterPIDFCoefficients();
                }
                telemetry.update();

                sleep(20);
            }
        });
        SecondaryThread.start();

        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        waitForStart();

        // ********* STARTED ********************************
        resetRuntime();
        Gericka_Hardware.autoTimeLeft = 0.0;


        double turretTargetAngleSmallTriangle = -115.0;
        theRobot.SetTurretRotationAngle(turretTargetAngleSmallTriangle);

        // turn on intake to suck in any stuck balls
        theRobot.SetIntakeMotor(true, true);

        // shooter speed for SMALL TRIANGLE
        double shooterSpeedRPM = 3500;
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // -------------------------
        // START -> SHOOT FROM SMALL TRIANGLE
        // -------------------------
        // Drive to the shooting position
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(DriveToShootingPosition));
        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(true, true);

        // SHOOT-3
        sleep(500);  // first time shooting give a tiny extra wait to allow shooter to spin up
        theRobot.ShootAutoThreeBalls();
        drive.updatePoseEstimate();

        final double BIG_TRIANGLE_RPM = 2800; // TODO: change this value
        double turretTargetAngleBigTriangle = -136.0; // TODO: change this value
        theRobot.SetLaunchRampPosition(0.6);

        shooterSpeedRPM = BIG_TRIANGLE_RPM;
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
        theRobot.SetTurretRotationAngle(turretTargetAngleBigTriangle);


        // -------------------------
        // SECOND STRIP -> OPEN-GATE -> BIG TRIANGLE -> SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(DriveToSecondMark, setIntakeOn(), new SetLifterDown()),
                        new SleepAction(0.4),
                        new ParallelAction(SecondMarkToLock, setIntakeOff()),
                        new SleepAction(0.1),
                        new ParallelAction(LockToBigTriangle)
                )
        );


        drive.updatePoseEstimate();
        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(true, true);

        // SHOOT-3
        theRobot.ShootAutoThreeBalls();

        // -------------------------
        // THIRD STRIP -> BIG TRIANGLE -> SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(DriveBigTriangleToThirdMark, setIntakeOn(), new SetLifterDown()),
                        new SleepAction(0.250),
                        new ParallelAction(DriveThirdMarkToBigTriangle, setIntakeOff())
                )
        );
        drive.updatePoseEstimate();

        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(true, true);
        // SHOOT-3
        theRobot.ShootAutoThreeBalls();

        // -------------------------
        // FIRST STRIP -> SMALL TRIANGLE -> SHOOT
        // -------------------------
        theRobot.SetLaunchRampPosition(1);
        theRobot.SetShooterMotorToSpecificRPM(3500);
        theRobot.SetTurretRotationAngle(-115);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(DriveBigTriangleToFirstMark, setIntakeOn(), new SetLifterDown()),
                        new SleepAction(0.250),
                        new ParallelAction(DriveFirstMarkToShootingPosition, setIntakeOff())
                )
        );
        drive.updatePoseEstimate();

        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(true, true);
        // SHOOT-3
        theRobot.ShootAutoThreeBalls();

        theRobot.SetTurretRotationAngle(0.0);

        // -------------------------
        // SMALL TRIANGLE -> PARK NEXT TO GATE
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(DriveShootingPositionToGateLock, setIntakeOff()));

        // -------------------------
        // Cleanup
        // -------------------------
        drive.updatePoseEstimate();
        blackboard.put(Gericka_Hardware.FINAL_X_POSITION, drive.localizer.getPose().position.x);
        blackboard.put(Gericka_Hardware.FINAL_Y_POSITION, drive.localizer.getPose().position.y);
        blackboard.put(Gericka_Hardware.FINAL_HEADING_DEGREES, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

        //lower lifter
        theRobot.SetLifterDown();
        theRobot.SetAutoLifterMode(false);

        // turn off shooter wheel
        theRobot.SetShooterMotorToSpecificRPM(0.0);

        // return turret to zero position
        theRobot.SetTurretRotationAngle(0.0);

        // turn off intake
        theRobot.SetIntakeMotor(true, true);

        Gericka_Hardware.autoTimeLeft = 30 - getRuntime();
        telemetry.addData("Time left", Gericka_Hardware.autoTimeLeft);
        telemetry.update();

        while ((getRuntime() < 29) && (!isStopRequested())) {
            sleep(20);
        }
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
            theRobot.SetIntakeMotor(true, true);
            packet.put("lock purple pixel", 0);
            return false;
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
            theRobot.SetIntakeMotor(false, false);
            packet.put("lock purple pixel", 0);
            return false;
        }
    }

    public class SetLifterUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            theRobot.SetLifterUp();
            packet.put("lock purple pixel", 0);
            return false;
        }
    }

    public class SetLifterDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
            }
            theRobot.SetLifterDown();
            packet.put("lock purple pixel", 0);
            return false;
        }
    }
}

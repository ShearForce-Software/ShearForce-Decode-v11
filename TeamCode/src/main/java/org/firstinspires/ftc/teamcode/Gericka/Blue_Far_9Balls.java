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
@Autonomous(name = "BLUE FAR 9 BALLS", preselectTeleOp = "Gericka 1 Manual Control")
public class

Blue_Far_9Balls extends LinearOpMode {

    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;

    @Override
    public void runOpMode() {
        final double startPoseHeadingDegrees = -90;
        Pose2d startPose = new Pose2d(60, -8.75, Math.toRadians(startPoseHeadingDegrees));  //TODO WHY was this changed from -8.75 like all of the other FAR routines back to -12 ???
        final double SMALL_TRIANGLE_RPM = 3000.0;
        //final double BIG_TRIANGLE_RPM = 2800;
        final double SMALL_TRIANGLE_TARGET_ANGLE = 117.0;
        //final double BIG_TRIANGLE_TARGET_ANGLE = 136.0;
        final double SMALL_TRIANGLE_HOOD_POSITION = 0.7;
        //final double BIG_TRIANGLE_HOOD_POSITION = 0.6;

        /* Initialize the Robot */
        theRobot.Init(hardwareMap, "BLUE");
        drive = new Gericka_MecanumDrive(hardwareMap, startPose);
        theRobot.InitRoadRunner(drive);
        theRobot.WebcamInit(this.hardwareMap);
        theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);
        theRobot.SetAutoLifterMode(true);
        theRobot.SetShooterPIDF_Enabled(true);
        Gericka_Hardware.shooterF = theRobot.PIDF_F_SMALL_TRIANGLE;

        // Turn turret toward the target
        double turretTargetAngle = SMALL_TRIANGLE_TARGET_ANGLE;
        theRobot.SetTurretRotationAngle(turretTargetAngle);
        theRobot.SetLaunchRampPosition(SMALL_TRIANGLE_HOOD_POSITION);

        sleep(3000); // allow turret to reach position
        // turn off turret power so it doesn't twitch during init
        theRobot.TurnOffTurret();

        // finish initializing pinpoint / roadrunner initial position
        sleep(1500);
        theRobot.SetRoadrunnerInitialPosition(startPose.position.x, startPose.position.y, startPoseHeadingDegrees);

        blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "BLUE");


        // ***************************************************
        // ****  Define Velocity and Acceleration Constraints
        // ***************************************************

        VelConstraint fastVel = new TranslationalVelConstraint(90);
        AccelConstraint fastAccel = new ProfileAccelConstraint(-65, 65);

        VelConstraint normalVel = new TranslationalVelConstraint(60);
        AccelConstraint normalAccel = new ProfileAccelConstraint(-30, 40);

        VelConstraint slowVel = new TranslationalVelConstraint(40);
        AccelConstraint slowAccel = new ProfileAccelConstraint(-25, 25);

        VelConstraint superSlowVel = new TranslationalVelConstraint(30);
        AccelConstraint superSlowAccel = new ProfileAccelConstraint(-15, 15);


        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        Action DriveToShootingPosition = drive.actionBuilder(new Pose2d(startPose.position.x, startPose.position.y, Math.toRadians(startPoseHeadingDegrees)))
                .strafeToConstantHeading(new Vector2d(48, -12), fastVel, fastAccel)
                .build();

        Action DriveToFirstMark = drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(34.75, -30), fastVel, fastAccel)
                //.splineToConstantHeading(new Vector2d(34.75, 60), Math.toRadians(90), intakeVel, intakeAccel)
                .strafeToConstantHeading(new Vector2d(34.75, -60), normalVel, normalAccel)
                .build();

        Action DriveFirstMarkToShootingPosition =  drive.actionBuilder(new Pose2d(34.75, -60, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(48, -12), fastVel, normalAccel)
                .build();

        Action DriveToSecondMark = drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(11.5, -30), Math.toRadians(-90), fastVel, fastAccel)
                .splineToConstantHeading(new Vector2d(11.5, -60), Math.toRadians(-90), normalVel, normalAccel)
                .build();

        Action DriveSecondMarkToShootingPosition = drive.actionBuilder(new Pose2d(11.5, -60, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(48, -12), fastVel, normalAccel)
                .build();

        Action DriveShootingPositionToGateLock =  drive.actionBuilder(new Pose2d(48, -12, Math.toRadians(-90)))
                .strafeToConstantHeading(new Vector2d(0, -30), fastVel, fastAccel)
                .build();

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

        // re-command the turret angle to get the power back on
        theRobot.SetTurretRotationAngle(turretTargetAngle);

        // turn on intake to suck in any stuck balls
        theRobot.SetIntakeMotor(true, true);

        // shooter speed for SMALL TRIANGLE
        double shooterSpeedRPM = SMALL_TRIANGLE_RPM;
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // -------------------------
        // START -> SHOOT FROM SMALL TRIANGLE
        // -------------------------
        // Drive to the shooting position
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(DriveToShootingPosition));
        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(false, true);
        Actions.runBlocking(new SleepAction(1));

        // SHOOT-3
        sleep(500);  // first time shooting give a tiny extra wait to allow shooter to spin up
        //theRobot.ShootAutoThreeBalls();
        theRobot.ShootAutoFourBalls();

                // -------------------------
                // -> FIRST STRIP -> SMALL TRIANGLE -> SHOOT
                // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(DriveToFirstMark, setIntakeOn(), new SetLifterDown()),
                        new SleepAction(0.250),
                        new ParallelAction(DriveFirstMarkToShootingPosition, setIntakeOff())
                )
        );
        drive.updatePoseEstimate();
        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(false, true);
        // SHOOT-3
        //theRobot.ShootAutoThreeBalls();
        theRobot.ShootAutoFourBalls();

                // -------------------------
                // SMALL TRIANGLE -> SECOND STRIP -> SMALL TRIANGLE -> SHOOT
                // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(DriveToSecondMark, setIntakeOn(), new SetLifterDown()),
                        new SleepAction(0.250),
                        new ParallelAction(DriveSecondMarkToShootingPosition, setIntakeOff())
                )
        );
        drive.updatePoseEstimate();

        // turn off intake to maximize power to the shooter
        theRobot.SetIntakeMotor(false, true);
        // SHOOT-3
        //theRobot.ShootAutoThreeBalls();
        theRobot.ShootAutoFourBalls();

        // Command Turret to start storing itself at 0
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
        theRobot.SetIntakeMotor(false, true);

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

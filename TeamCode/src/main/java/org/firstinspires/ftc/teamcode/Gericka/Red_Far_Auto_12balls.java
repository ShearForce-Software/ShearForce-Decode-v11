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
@Autonomous(name = "RED Far Auto 12 balls", preselectTeleOp = "Gericka 1 Manual Control")
public class Red_Far_Auto_12balls extends LinearOpMode {

    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;

    // Trajectories
    Action DriveToShootingPosition;

    Action DriveToFirstMark;
    Action ReturnFromFirstMark;

    Action DriveToSecondMark;
    Action ReturnFromSecondMark;

    Action DriveToThirdMark;
    Action DriveThirdMarkToBigTriangle;
    Action DriveToGateLock;

    // Constraints
    VelConstraint fastVel = new TranslationalVelConstraint(85);
    AccelConstraint fastAccel = new ProfileAccelConstraint(-60, 60);

    VelConstraint intakeVel = new TranslationalVelConstraint(60);
    AccelConstraint intakeAccel = new ProfileAccelConstraint(-30, 40);

    public static int lifterUpSleepTime = 300;
    public static int lifterDownSleepTime = 400;

    @Override
    public void runOpMode() {

        startPose = new Pose2d(60, 12, Math.toRadians(90));

        /* Initialize the Robot */
        theRobot.Init(hardwareMap, "RED");

        // initialize roadrunner
        drive = new Gericka_MecanumDrive(hardwareMap, startPose);
        theRobot.InitRoadRunner(drive);


        // finish initializing pinpoint / roadrunner initial position
        sleep(500);
        theRobot.SetRoadrunnerInitialPosition(60, 12, 90);

        blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "RED");

        // set lifter half up (so can get 3 balls loaded in robot)
        theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        DriveToShootingPosition = drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                .build();

        // FIRST  = far-right strip (closest to GOAL side)
        // SECOND = center strip
        // THIRD  = far-left strip

        DriveToFirstMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))
                // Smooth into the strip (fast -> intake). Tangent set to +Y so it flows into the upfield run.
                .splineToConstantHeading(new Vector2d(34.75, 30), Math.toRadians(90), fastVel, fastAccel)
                .splineToConstantHeading(new Vector2d(34.75, 63), Math.toRadians(90), intakeVel, intakeAccel)
                .build();

        ReturnFromFirstMark = drive.actionBuilder(new Pose2d(34.75, 63, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                .build();

        DriveToSecondMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))

                .splineToConstantHeading(new Vector2d(11.5, 30), Math.toRadians(90), fastVel, fastAccel)
                .splineToConstantHeading(new Vector2d(11.5, 63), Math.toRadians(90), intakeVel, intakeAccel)
                .build();

        ReturnFromSecondMark = drive.actionBuilder(new Pose2d(11.5, 63, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                .build();

        DriveToThirdMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))

                .splineToConstantHeading(new Vector2d(-15, 32), Math.toRadians(90), fastVel, fastAccel)
                .splineToConstantHeading(new Vector2d(-15, 60), Math.toRadians(90), intakeVel, intakeAccel)
                .build();


        DriveThirdMarkToBigTriangle = drive.actionBuilder(new Pose2d(-15, 60, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-11.5, 11.5), fastVel, fastAccel)
                .build();


        DriveToGateLock = drive.actionBuilder(new Pose2d(-11.5, 11.5, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, 46), fastVel, fastAccel)
                .build();

        theRobot.SetAutoLifterMode(true);

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


        theRobot.SetShooterPIDF_Enabled(false);
        Gericka_Hardware.shooterF = theRobot.PIDF_F_SMALL_TRIANGLE;
        theRobot.SetShooterPIDFCoefficients();

        // TODO: Set turret angle for SMALL TRIANGLE shots (edit this)
        double turretTargetAngleSmallTriangle = -115.0;
        theRobot.SetTurretRotationAngle(turretTargetAngleSmallTriangle);

        // turn on intake to suck in any stuck balls
        theRobot.SetIntakeMotor(true, true);

        // shooter speed for SMALL TRIANGLE
        final double SMALL_TRIANGLE_RPM = 3400;
        double shooterSpeedRPM = SMALL_TRIANGLE_RPM;
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // Drive to the shooting position
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(DriveToShootingPosition, setIntakeOff()));
        drive.updatePoseEstimate();

        // Shoot the 3 preloaded balls
        ShootBall(shooterSpeedRPM);
        ShootBall(shooterSpeedRPM);
        ShootBall(shooterSpeedRPM);

        // -------------------------
        // FIRST STRIP -> BACK -> SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(DriveToFirstMark, setIntakeOn(), new SetLifterDown()),
                        new ParallelAction(ReturnFromFirstMark, setIntakeOff())
                )
        );
        drive.updatePoseEstimate();

        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true, true);
        ShootBall(shooterSpeedRPM);
        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true, true);

        // -------------------------
        // SECOND STRIP -> BACK -> SHOOT
        // -------------------------
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(DriveToSecondMark, setIntakeOn(), new SetLifterDown()),
                        new ParallelAction(ReturnFromSecondMark, setIntakeOff())
                )
        );
        drive.updatePoseEstimate();

        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true, true);
        ShootBall(shooterSpeedRPM);
        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true, true);

        // -------------------------
        // THIRD STRIP -> BIG TRIANGLE -> SHOOT
        // -------------------------


        final double BIG_TRIANGLE_RPM = 2700; // TODO: change this value
        double turretTargetAngleBigTriangle = -142.0; // TODO: change this value

        shooterSpeedRPM = BIG_TRIANGLE_RPM;
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
        theRobot.SetTurretRotationAngle(turretTargetAngleBigTriangle);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(DriveToThirdMark, setIntakeOn(), new SetLifterDown()),
                        new ParallelAction(DriveThirdMarkToBigTriangle, setIntakeOff())
                )
        );
        drive.updatePoseEstimate();

        // Shoot last 3 balls from big triangle
        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true, true);
        ShootBall(shooterSpeedRPM);
        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true, true);


        theRobot.SetTurretRotationAngle(0.0);

        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(DriveToGateLock, setIntakeOff()));
        drive.updatePoseEstimate();

        // -------------------------
        // Cleanup
        // -------------------------
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

    private void ShootBall(double shooterSpeedRPM) {
        theRobot.SetLifterUp();
        sleep(lifterUpSleepTime);

        theRobot.SetLifterDown();
        sleep(lifterDownSleepTime);
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

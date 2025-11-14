package org.firstinspires.ftc.teamcode.Gericka;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//TODO auto select manual opMode next
@Autonomous(name="Red Far Auto", preselectTeleOp ="")

public class Red_Far_Auto extends LinearOpMode {
    Gericka_Hardware control = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;
    public static final String ALLIANCE_KEY = "Alliance";

    // Trajectories

    Action DriveToSecondMark;
    Action DriveSecondMarkToSmallTriangle;
    Action DriveToFirstMark;
    Action DriveFirstMarkToSmallTriangle;
    Action DriveOutofLaunchZone;

    public void runOpMode() {
        startPose = new Pose2d(60, 12, Math.toRadians(90));
        /* Initialize the Robot */
        drive = new Gericka_MecanumDrive(hardwareMap, startPose);

        control.Init(hardwareMap, "RED");
        blackboard.put(ALLIANCE_KEY, "RED");

        //TODO turn turret to face the obolisk
        double turretTargetAngle = -91.0;
        control.SetTurretRotationAngle(turretTargetAngle);

        // set lifter half up (so can get 3 ball loaded in robot)
        control.SetLifterPosition(control.LIFTER_MID_POSITION);

        telemetry.update();

        //control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        // *** Route aligns with Decode_MeepMeep_Blue_ID22_Small_Triangle ***

        DriveToSecondMark = drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(11.5, 30))
                .strafeToConstantHeading(new Vector2d(11.5, 55))
                .build();

        DriveSecondMarkToSmallTriangle = drive.actionBuilder(new Pose2d(11.5, 55, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(20, 12))
                .strafeToConstantHeading(new Vector2d(60, 12))
                .build();

        DriveToFirstMark = drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(34.75, 30))
                .strafeToConstantHeading(new Vector2d(34.75, 55))
                .build();

        DriveFirstMarkToSmallTriangle = drive.actionBuilder(new Pose2d(34.75, 55, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(60, 12))
                .build();

        DriveOutofLaunchZone = drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(20, 12))
                .build();

        Thread SecondaryThread = new Thread(() -> {
            while (!isStopRequested() && getRuntime() < 30) {
                //control.ShowTelemetry();

                control.ShowPinpointTelemetry();

                telemetry.update();

                sleep(20);
            }
        });
        SecondaryThread.start();

        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        while (!isStarted()) {
            telemetry.update();
        }

        // ********* STARTED ********************************
        resetRuntime();
        Gericka_Hardware.autoTimeLeft = 0.0;

        //TODO Read the obsolisk apriltag, display result in telemetry (we don't really need it yet, but should start assessing our ability to get it)

        //TODO Turn Turret towards target, can leave turret there the whole time
        turretTargetAngle = -115.0;
        control.SetTurretRotationAngle(turretTargetAngle);

        //TODO determine optimum speed for small triangle shots probably 3500rpm, can just leave at this speed the whole time
        double shooterSpeedRPM = 4500;
        control.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
        sleep(4000);

        // shoot the 3 pre-loaded balls
        control.SetLifterUp();  // shoot ball 1
        sleep(1000);
        control.SetLifterDown();
        sleep(1000);
        control.SetLifterUp();  // shoot ball 2
        sleep(1000);
        control.SetLifterDown();
        sleep(1000);
        control.SetLifterUp();  // shoot ball 3
        sleep(1000);
        control.SetLifterDown();

        shooterSpeedRPM = 3500;
        control.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        //TODO create a separate thread that auto lifts the lifter half way up when ball detected


        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        //TODO add parallel action that turns intake on
                        new ParallelAction(DriveToSecondMark, setIntakeOn()),
                        new SleepAction(1.0), // tiny sleep to finish ingesting balls, not sure how much is really needed
                         //TODO add parallel action that turns intake off
                        new ParallelAction(DriveSecondMarkToSmallTriangle, setIntakeOff()),
                        //TODO shoot 3 here with sequential action set (lift up, sleep briefly, drop lifter, sleep briefly, lift up, sleep briefly, drop lifter, sleep briefly, lift up, sleep briefly)
                        //new SleepAction(2),

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

                            //TODO add parallel action that turns intake on
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

        //TODO store final exact position in blackboard, so can initialize manual pinpoint with that position

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

package org.firstinspires.ftc.teamcode.Gericka;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Close Auto", preselectTeleOp ="Gericka 1 Manual Control")


public class Blue_Close_Auto extends LinearOpMode {
    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;

    // Trajectories
    Action DriveStartToMidPosition;
    Action DriveMidToClosestLine;
    Action DriveClosestLineBackToLaunchPark;
    int lifterUpSleepTime = 500;
    int lifterDownSleepTime = 600;

    public void runOpMode(){
    //We will start at big trianlge start
    startPose = new Pose2d(-60,-39,Math.toRadians(270));
    /* Initialize the Robot */
    theRobot.Init(hardwareMap, "BLUE");

    // initialize roadrunner
    drive = new Gericka_MecanumDrive(hardwareMap, startPose);
    theRobot.InitRoadRunner(drive);

    // initialize the webcam
    theRobot.WebcamInit(this.hardwareMap);

    sleep(500); // sleep at least 1/4 second to allow pinpoint to calibrate itself
    // finish initializing the pinpoint
    theRobot.SetInitalPinpointPosition(-60, -39, 270);

    blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "BLUE");

    // set lifter half up (so can get 3 ball loaded in robot)
    theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);


    DriveStartToMidPosition = drive.actionBuilder(new Pose2d(-60, -39, Math.toRadians(270)))
            .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(210))
            .turnTo(Math.toRadians(270))
            .build();

    // Mid (-10, -10, 270) -> closest line (-10, -40)
    DriveMidToClosestLine = drive.actionBuilder(new Pose2d(-10, -10, Math.toRadians(270)))
            .strafeToConstantHeading(new Vector2d(-10, -40))
            .build();

    // Closest line (-10, -40, 270) -> launch park (-54, -16)
    DriveClosestLineBackToLaunchPark = drive.actionBuilder(new Pose2d(-10, -40, Math.toRadians(270)))
            .strafeToConstantHeading(new Vector2d(-54, -16))
            .build();

        theRobot.SetAutoLifterMode(true);

    // ***************************************************
    // ****  Secondary Thread to run all the time ********
    // ***************************************************
    Thread secondaryThread = new Thread(() -> {
        while (!isStopRequested() && getRuntime() < 30) {
                theRobot.ShowTelemetry();
                //control.ShowPinpointTelemetry();

                if (isStarted()) {
                    theRobot.RunAutoLifter();
                }
            telemetry.update();
            sleep(20);
        }
    });
    secondaryThread.start();

    // Turret initial rough angle toward speaker (tune as needed)
    double turretTargetAngle = 135.8;
    theRobot.SetTurretRotationAngle(turretTargetAngle);
    sleep(1000);
    // turn off turret power so doesn't twitch
    theRobot.turretMotor.setPower(0);

    // *************************************
    //      Wait for start
    // *************************************
        waitForStart();

    // ********* STARTED ********************************
    resetRuntime();
    Gericka_Hardware.autoTimeLeft = 0.0;

    // turn on intake to suck in any stuck balls
    theRobot.SetIntakeMotor(true,true);

    // spin up shooter wheel to max
    theRobot.SetShooterSpeed(1.0);


    Actions.runBlocking(new SleepAction((2.0)));
    Actions.runBlocking(DriveStartToMidPosition);

    // Turn turret more directly to target for auto shooting (tune on field)
    //turretTargetAngle = 45;    // CHANGE LLATER
    //control.SetTurretRotationAngle(turretTargetAngle);

    // Shooter RPM for big triangle shots (tune as needed)
    double shooterSpeedRPM = 2900;
    theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
        // Turn on intake incase a ball is stuck
        theRobot.SetIntakeMotor(true, true);

        /* **** SHOOT BALL #1 **** */
        ShootBall(shooterSpeedRPM);

        // Turn off intake to save energy
        theRobot.SetIntakeMotor(false, true);

        /* **** SHOOT BALL #2 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #3 **** */
        ShootBall(shooterSpeedRPM);

    //Should we turn intake on while we go to the closest line
    theRobot.SetIntakeMotor(true, true);


    Actions.runBlocking(
            new SequentialAction(
                    // Mid -> closest line
                    DriveMidToClosestLine,
                    new SleepAction(0.5),           // tiny sleep to finish ingesting rings
                    // Closest line -> launch park
                    DriveClosestLineBackToLaunchPark,
                    // Final 2 second wait to match MeepMeep .waitSeconds(2) at end
                    new SleepAction(2.0)
            )
    );

    theRobot.SetIntakeMotor(false, true);


   // sleep(5000);//giving it sleepy time

        /* **** SHOOT BALL #4 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #5 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #6 **** */
        ShootBall(shooterSpeedRPM);

        //TODO need to park away from the line

        // store final exact position in blackboard, so can initialize absolute pinpoint with that position
        //control.pinpoint.update();
        drive.updatePoseEstimate();
        blackboard.put(Gericka_Hardware.FINAL_X_POSITION, drive.localizer.getPose().position.x);
        blackboard.put(Gericka_Hardware.FINAL_Y_POSITION, drive.localizer.getPose().position.y);
        blackboard.put(Gericka_Hardware.FINAL_HEADING_DEGREES, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

    // turn off shooter wheel
    shooterSpeedRPM = 0.0; //3200rpm was about the value observed when the Motor was commanded to 75%.
    theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

    // return turret to zero position
    turretTargetAngle = 0.0;
    theRobot.SetTurretRotationAngle(turretTargetAngle);

    Gericka_Hardware.autoTimeLeft = 30-getRuntime();
    telemetry.addData("Time left", Gericka_Hardware.autoTimeLeft);
    telemetry.update();


}

    private void ShootBall(double shooterSpeedRPM) {
        // sleep some time to allow shooter wheel to spin back up if needed
        while (theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 10) ||
                theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) > (shooterSpeedRPM + 10)) {
            sleep(20);
        }

        /* **** SHOOT a BALL  **** */
        theRobot.SetLifterUp();
        sleep(lifterUpSleepTime);

        // Reset to get another ball
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
                theRobot.SetIntakeMotor(false, false);
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
            theRobot.SetLifterUp();
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
            theRobot.SetLifterDown();
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

}

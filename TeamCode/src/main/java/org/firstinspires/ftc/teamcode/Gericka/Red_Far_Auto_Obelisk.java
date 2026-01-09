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
@Autonomous(name="RED Far Auto obelisk", preselectTeleOp ="Gericka 1 Manual Control")

public class Red_Far_Auto_Obelisk extends LinearOpMode {
    Gericka_Hardware theRobot = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;

    boolean changeTurretAngleFirstPass;
    boolean changeTurretAngleSecondPass;
    boolean changeTurretAngleThirdPass;


    // Trajectories


    Action DriveToShootingPosition;
    Action DriveSecondMarkToSmallTriangle;
    Action DriveToFirstMark;
    Action DriveToSecondMark;
    Action DriveToThirdMark;
    Action DriveFirstMarkToSmallTriangle;
    Action DriveOutofLaunchZone;
    Action ReturnFromFirstMark;
    Action ReturnFromSecondMark;
    Action ReturnFromThirdMark;

    Action DriveToSecondMarkFromLargeTriangle;
    Action DriveThirdMarkToLargeTriangle;
    Action DriveLargetriangleToSecondMark;

    Action DriveSecondMarkToLargeTriangle;

    Action DriveLargeTriangleToFirstMark;


    Action DriveOutofLaunchZoneForFirstMark;
    Action DriveOutofLaunchZoneForSecondMark;
    Action DriveOutofLaunchZoneForThirdMark;


    VelConstraint fastVel = new TranslationalVelConstraint(85);
    AccelConstraint fastAccel = new ProfileAccelConstraint(-60, 60);

    VelConstraint intakeVel = new TranslationalVelConstraint(45);
    AccelConstraint intakeAccel = new ProfileAccelConstraint(-30, 40);

    //Action ReturnFromThirdMarkSecond;
    //Action ReturnFromSecondMarkSecond;
    //Action ReturnFromFirstMarkSecond;



    int lifterUpSleepTime = 500; //300 works very well, can probably go lower to 200
    int lifterDownSleepTime = 600; //400 works well, can probably go lower to 300 maybe 200
    //Add constraints here


    public enum SampleLine{
        FIRST, //
        SECOND,
        THIRD,
        IDK;
    }



    public SampleLine getSampleLineForObeliskId(int id, String alliance) {
        switch (id) {
            case 21:
                return SampleLine.FIRST; // GPP
            case 22:
                return SampleLine.SECOND; // PGP
            case 23:
                return SampleLine.THIRD; // PPG
        }
        return SampleLine.IDK;
    }





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
        double turretTargetAngle = -91.0;
        theRobot.SetTurretRotationAngle(turretTargetAngle);
        theRobot.SetLaunchRampPosition(1.0);

        sleep(3000); // allow turret to reach position
        // turn off turret power so it doesn't twitch during init
        theRobot.TurnOffTurret();

        // Now scan for the obelisk motif for up to 3 seconds
        int obeliskId = theRobot.detectObeliskMotif(3000);
       // telemetry.addData("Final Obelisk ID", obeliskId);

        SampleLine line = getSampleLineForObeliskId(obeliskId, "RED");

       // telemetry.addData("Final Obeelisk ID", obeliskId);
        //telemetry.addData("Final Obelisk ID", obeliskId);
        //telemetry.addData("Sample Line", line);
        //telemetry.update();

        sleep(500); // sleep at least 1/4 second to allow pinpoint to calibrate itself
        // finish initializing the pinpoint
        theRobot.SetRoadrunnerInitialPosition(60, 12, 90);

        blackboard.put(Gericka_Hardware.ALLIANCE_KEY, "RED");

        // set lifter half up (so can get 3 ball loaded in robot)
        theRobot.SetLifterPosition(theRobot.LIFTER_MID_POSITION);

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************



        DriveToShootingPosition = drive.actionBuilder(new Pose2d(60, 12, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12))
                .build();

// FIRST  = far-right strip (closest to GOAL side)
// SECOND = center strip
// THIRD  = far-left strip

        DriveToFirstMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(34.75, 30), fastVel, fastAccel)
                .strafeToConstantHeading(new Vector2d(34.75, 63), intakeVel, intakeAccel)
                .build();

        DriveToSecondMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(11.5, 30), fastVel, fastAccel)
                .strafeToConstantHeading(new Vector2d(11.5, 63), intakeVel, intakeAccel)
                .build();

        DriveToThirdMark = drive.actionBuilder(new Pose2d(48, 12, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(-15, 32), fastVel, fastAccel)
                .strafeToConstantHeading(new Vector2d(-15, 60), intakeVel, intakeAccel)
                .build();

        ReturnFromFirstMark = drive.actionBuilder(new Pose2d(34.75, 63, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                .build();


        ReturnFromSecondMark = drive.actionBuilder(new Pose2d(11.5, 63, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(20, 12), fastVel, fastAccel)
                .strafeToConstantHeading(new Vector2d(48, 12), fastVel, fastAccel)
                .build();

        DriveThirdMarkToLargeTriangle = drive.actionBuilder(new Pose2d(-15, 60, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-12.4, 12.7), fastVel, fastAccel)
                .build();

        DriveLargetriangleToSecondMark = drive.actionBuilder(new Pose2d(-12.4, 23.4, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(11.5, 30), fastVel, fastAccel)
                .strafeToConstantHeading(new Vector2d(11.5, 63), intakeVel, intakeAccel)
                .build();

        DriveLargeTriangleToFirstMark = drive.actionBuilder(new Pose2d(-23.7, 23.4, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(34.75, 30), fastVel, fastAccel)
                .strafeToConstantHeading(new Vector2d(34.75, 63), intakeVel, intakeAccel)
                .build();


        DriveSecondMarkToLargeTriangle = drive.actionBuilder(new Pose2d(11.5, 63, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(-12.4, 12.7), fastVel, fastAccel)
                .build();





        theRobot.SetAutoLifterMode(true);

        // ***************************************************
        // ****  Secondary Thread to run all the time ********
        // ***************************************************
        Thread SecondaryThread = new Thread(() -> {
            while (!isStopRequested() && getRuntime() < 30) {
                theRobot.ShowTelemetry();
                //control.ShowPinpointTelemetry();

                theRobot.SetIndicatorLights();

                if (isStarted()) {
                    theRobot.RunAutoLifter();
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

        //if(obeliskId==-1){
            obeliskId = theRobot.detectObeliskMotif(1500);
            line = getSampleLineForObeliskId(obeliskId, "RED");
        //}
       // telemetry.addData("Final Obelisk ID", obeliskId);

        theRobot.SetShooterPIDF_Enabled(false);
        Gericka_Hardware.shooterF = theRobot.PIDF_F_SMALL_TRIANGLE;
        theRobot.SetShooterPIDFCoefficients();

        // Turn Turret towards target, can leave turret there the whole time
        turretTargetAngle = -115.0;
        theRobot.SetTurretRotationAngle(turretTargetAngle);

        // turn on intake to suck in any stuck balls
        theRobot.SetIntakeMotor(true,true);

        // set shooter speed to small triangle speed, can just leave at this speed the whole time

        //final double SMALL_TRIANGLE_RPM = 3400;
        //nal double BIG_TRIANGLE_RPM = 3100;
        //double shooterSpeedRPM = (line== SampleLine.THIRD) ? BIG_TRIANGLE_RPM : SMALL_TRIANGLE_RPM;
        double shooterSpeedRPM = 3400;
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

        // drive to the start triangle
        drive.updatePoseEstimate();
        Actions.runBlocking(new SequentialAction(DriveToShootingPosition, setIntakeOff()));
        drive.updatePoseEstimate();

        /* **** SHOOT BALL #1 **** */
        ShootBall(shooterSpeedRPM);

        // can stop the intake after the first shot to save power
        //control.SetIntakeMotor(false,true);

        /* **** SHOOT BALL #2 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #3 **** */
        ShootBall(shooterSpeedRPM);

        Action pickLineAction;
        Action returnFromPickedLineAction;
        Action repeatPickLineAction;
        Action returnFromRepeatPickedLineAction;
        Action driveOutOfLaunch;
        Action repeatThirdPickLineAction = new SleepAction(0);
        Action returnFromThirdPickLineAction = new SleepAction(0);



        switch(line){
            case FIRST:
                pickLineAction = DriveToFirstMark;
                returnFromPickedLineAction = ReturnFromFirstMark;
                repeatPickLineAction = DriveToSecondMark;
                returnFromRepeatPickedLineAction = ReturnFromSecondMark;
                repeatThirdPickLineAction = DriveToThirdMark;
                returnFromThirdPickLineAction = DriveThirdMarkToLargeTriangle;
                changeTurretAngleFirstPass = false;
                changeTurretAngleSecondPass = false;
                changeTurretAngleThirdPass = true;

                break;
            case SECOND:
                pickLineAction = DriveToSecondMark;
                returnFromPickedLineAction = ReturnFromSecondMark;
                repeatPickLineAction = DriveToFirstMark;
                returnFromRepeatPickedLineAction = ReturnFromFirstMark;
                repeatThirdPickLineAction = DriveToThirdMark;
                returnFromThirdPickLineAction = DriveThirdMarkToLargeTriangle;
                changeTurretAngleFirstPass = false;
                changeTurretAngleSecondPass = false;
                changeTurretAngleThirdPass = true;

                break;
            case THIRD:
                pickLineAction = DriveToThirdMark;
                returnFromPickedLineAction = DriveThirdMarkToLargeTriangle;
                repeatPickLineAction = DriveLargetriangleToSecondMark;
                returnFromRepeatPickedLineAction = DriveSecondMarkToLargeTriangle;
                repeatThirdPickLineAction = DriveLargeTriangleToFirstMark;
                returnFromThirdPickLineAction = ReturnFromFirstMark;
                changeTurretAngleFirstPass = true;
                changeTurretAngleSecondPass = true;
                changeTurretAngleThirdPass = false;

                break;
            default:
                pickLineAction = DriveToFirstMark;
                returnFromPickedLineAction=ReturnFromFirstMark;
                repeatPickLineAction = DriveToSecondMark;
                returnFromRepeatPickedLineAction = ReturnFromSecondMark;
                repeatThirdPickLineAction = DriveToThirdMark;
                returnFromThirdPickLineAction = DriveThirdMarkToLargeTriangle;
                changeTurretAngleFirstPass = false;
                changeTurretAngleSecondPass = false;
                changeTurretAngleThirdPass = true;

                break;
        }
        // If you turned turret you are obviously in closer big triangle and need to lower speed
        if(changeTurretAngleFirstPass){
            theRobot.SetTurretRotationAngle(-142);
        }
        else{
            theRobot.SetTurretRotationAngle(-91);
        }



        // Drive to middle line, get balls, and return to launch zone
        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        // Drive to the middle line and turn the intake on
                        new ParallelAction(pickLineAction, setIntakeOn()),
                        //new SleepAction(1.0), // tiny sleep to finish ingesting balls, not sure how much is really needed
                        // Return to launch zone and turn intake off
                        new ParallelAction(returnFromPickedLineAction, setIntakeOff())
                ));
        drive.updatePoseEstimate();

        /* **** SHOOT BALL #4 **** */
        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true,true);

        /* **** SHOOT BALL #5 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #6 **** */
        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true,true);



        if(changeTurretAngleSecondPass){
            theRobot.SetTurretRotationAngle(-142);
        }
        else{
            theRobot.SetTurretRotationAngle(-91);
        }


        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        // Drive to closest line and turn intake on and lower lifter
                        new ParallelAction(repeatPickLineAction, setIntakeOn(), new SetLifterDown()),
                        //new SleepAction(1.0) // tiny sleep to finish ingesting balls, not sure how much is really needed
                        new ParallelAction(returnFromRepeatPickedLineAction, setIntakeOff())
                ));
        drive.updatePoseEstimate();



            /* **** SHOOT BALL #7 **** */
            ShootBall(shooterSpeedRPM);
            theRobot.SetIntakeMotor(true,true);

            /* **** SHOOT BALL #8 **** */
            ShootBall(shooterSpeedRPM);

            /* **** SHOOT BALL #9 **** */
            ShootBall(shooterSpeedRPM);
            theRobot.SetIntakeMotor(true,true);
            // Drive to Parking spot

            drive.updatePoseEstimate();




        if(changeTurretAngleThirdPass){
            theRobot.SetTurretRotationAngle(-142);
        }
        else{
            theRobot.SetTurretRotationAngle(-91);
        }



        drive.updatePoseEstimate();
        Actions.runBlocking(
                new SequentialAction(
                        // Drive to closest line and turn intake on and lower lifter
                        new ParallelAction(repeatThirdPickLineAction, setIntakeOn(), new SetLifterDown()),
                        //new SleepAction(1.0) // tiny sleep to finish ingesting balls, not sure how much is really needed
                       new ParallelAction(returnFromThirdPickLineAction, setIntakeOff())
                ));
        drive.updatePoseEstimate();



        /* **** SHOOT BALL #10 **** */
        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true,true);

        /* **** SHOOT BALL #11 **** */
        ShootBall(shooterSpeedRPM);

        /* **** SHOOT BALL #12 **** */
        ShootBall(shooterSpeedRPM);
        theRobot.SetIntakeMotor(true,true);
        // Drive to Parking spot
        turretTargetAngle = 0;
        theRobot.SetTurretRotationAngle(turretTargetAngle);
        drive.updatePoseEstimate();





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

        // turn off intake to suck in any stuck balls
        theRobot.SetIntakeMotor(false,true);

        Gericka_Hardware.autoTimeLeft = 30 - getRuntime();
        telemetry.addData("Time left", Gericka_Hardware.autoTimeLeft);
        telemetry.update();

        while ((getRuntime() < 29) && (!isStopRequested() )){
            sleep(20);
        }

    }

    private void ShootBall(double shooterSpeedRPM) {
        double timeout = getRuntime() + 2.0; // only allow a max of 2 seconds delay for spin up waiting
        // sleep some time to allow shooter wheel to spin back up if needed
/*
        // while neither motor is at least within 200 of our target speed
        while ((theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 200)) &&
                (theRobot.CalculateMotorRPM(theRobot.shooterMotorRight.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 200)) &&
                (getRuntime() < timeout)) {
            theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM-100);
            sleep(20);
        }
        while ((theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 100)) &&
                (theRobot.CalculateMotorRPM(theRobot.shooterMotorRight.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 100) ) &&
                (getRuntime() < timeout)) {
            theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM-50);
            sleep(20);
        }
        while ((theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 10) ||
                theRobot.CalculateMotorRPM(theRobot.shooterMotorLeft.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) > (shooterSpeedRPM + 200)) &&
                (theRobot.CalculateMotorRPM(theRobot.shooterMotorRight.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) < (shooterSpeedRPM - 10) ||
                        theRobot.CalculateMotorRPM(theRobot.shooterMotorRight.getVelocity(), theRobot.YELLOW_JACKET_1_1_TICKS) > (shooterSpeedRPM + 200)) &&
                (getRuntime() < timeout)) {
            theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
            sleep(20);
        }
        theRobot.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
*/
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
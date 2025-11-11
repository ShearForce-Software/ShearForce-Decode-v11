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

import org.firstinspires.ftc.teamcode.Gericka.Gericka_Hardware;
import org.firstinspires.ftc.teamcode.Gericka.Gericka_MecanumDrive;

//TODO auto select manual opMode next
@Autonomous(name="Blue Far Auto", preselectTeleOp ="")

public class Blue_Far_Auto extends LinearOpMode {
    Gericka_Hardware control = new Gericka_Hardware(false, false,this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;
    public static final String ALLIANCE_KEY = "Alliance";

    // Trajectories

    Action DriveToSecondMark;
    Action DriveSecondMarkToSmallTriangle;
    Action DriveToFirstMark;
    Action DriveFirstMarkToSmallTriangle;
    Action DriveOutofLaunchZone;

    public void runOpMode(){
        startPose = new Pose2d(9,-64, Math.toRadians(270));
        /* Initialize the Robot */
        drive = new Gericka_MecanumDrive(hardwareMap, startPose);

        control.Init(hardwareMap, "BLUE");
        blackboard.put(ALLIANCE_KEY, "BLUE");

        //TODO turn turret to face the obolisk

        //TODO set lifter half up (so can get 3 ball loaded in robot)

        telemetry.update();

        //control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        // *** Route aligns with Decode_MeepMeep_Blue_ID22_Small_Triangle ***

        DriveToSecondMark = drive.actionBuilder(new Pose2d(60, -12, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(11.5,-30))
                        .strafeToConstantHeading(new Vector2d(11.5,-48))
                .build();

        DriveSecondMarkToSmallTriangle = drive.actionBuilder(new Pose2d(11.5,-48, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(20,-12))
                        .strafeToConstantHeading(new Vector2d(60,-12))
                        .build();

        DriveToFirstMark = drive.actionBuilder(new Pose2d(60,-12, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(34.75,-30))
                        .strafeToConstantHeading(new Vector2d(34.75,-48))
                        .build();

        DriveFirstMarkToSmallTriangle = drive.actionBuilder(new Pose2d(34.75,-48, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(60,-12))
                        .build();

        DriveOutofLaunchZone = drive.actionBuilder(new Pose2d(60,-12,Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(20,-12))
                        .build();

        // ***************************************************
        // ****  WAIT for START/PLAY to be pushed ************
        // ***************************************************
        while(!isStarted()){
            telemetry.update();
        }

        // ********* STARTED ********************************
        resetRuntime();
        Gericka_Hardware.autoTimeLeft = 0.0;

        //TODO Read the obsolisk apriltag, display result in telemetry (we don't really need it yet, but should start assessing our ability to get it)

        //TODO Turn Turret towards target, probably should have separate thread tracking the target, but could start by just picking correct turret position and leaving it there the whole time

        //TODO spin up shooter wheel to optimum speed for small triangle shots probably 3500rpm, can just leave at this speed the whole time

        //TODO create a separate thread that auto lifts the lifter half way up when ball detected


        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(2), //TODO shoot 3 here with sequential action set (lift up, sleep briefly, drop lifter, sleep briefly, lift up, sleep briefly, drop lifter, sleep briefly, lift up, sleep briefly)
                        DriveToSecondMark,     //TODO add parallel action that turns intake on
                        new SleepAction(0.5), // tiny sleep to finish ingesting balls, not sure how much is really needed
                        DriveSecondMarkToSmallTriangle, //TODO add parallel action that turns intake off
                        new SleepAction(2), //TODO shoot 3 here
                        DriveToFirstMark,       //TODO add parallel action that turns intake on
                        new SleepAction(0.5), // tiny sleep to finish ingesting balls, not sure how much is really needed
                        DriveFirstMarkToSmallTriangle, //TODO add parallel action that turns intake off
                        new SleepAction(2), //TODO shoot 3 here
                        DriveOutofLaunchZone));

        drive.updatePoseEstimate();

        //TODO store final exact position in blackboard, so can initialize manual pinpoint with that position

        //TODO turn off shooter wheel

        //TODO return shooter to zero position

        Gericka_Hardware.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", Gericka_Hardware.autoTimeLeft);
        telemetry.update();

    }
}

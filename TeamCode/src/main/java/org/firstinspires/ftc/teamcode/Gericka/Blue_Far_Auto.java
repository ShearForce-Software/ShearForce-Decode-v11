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


@Autonomous(name="Blue Far Auto", preselectTeleOp ="")

public class Blue_Far_Auto extends LinearOpMode {
    Gericka_Hardware control = new Gericka_Hardware(true, false,this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;
    public static final String ALLIANCE_KEY = "Alliance";

    // Trajectories

    Action DriveToSecondMark;
    Action DriveToFirstMark;
    Action DriveOutofLaunchZone;

    public void runOpMode(){
        startPose = new Pose2d(9,-64, Math.toRadians(270));
        /* Initialize the Robot */
        drive = new Gericka_MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap, "BLUE");
        blackboard.put(ALLIANCE_KEY, "BLUE");
        //  -- need to adjust this starting position to keep the specimen out of the wall Check
        //control.AutoStartPosition();
        //control.HooksReleased();
        //control.SetSwiperPosition(Gericka_Hardware.SWIPER_MAX_POS);
        //control.SetSwiper2Position(Gericka_Hardware.SWIPER2_MIN_POS);
        telemetry.update();
        //control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());

        // ***************************************************
        // ****  Define Trajectories    **********************
        // ***************************************************

        DriveToSecondMark = drive.actionBuilder(new Pose2d(60, -12, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(11.5,-30))
                        .strafeToConstantHeading(new Vector2d(11.5,-48))
                        //.strafeToLinearHeading(new Vector2d(0,-48),Math.toRadians(0))
                        //.strafeToConstantHeading(new Vector2d(0,-52))
                        .strafeToConstantHeading(new Vector2d(20,-12))
                        .strafeToConstantHeading(new Vector2d(60,-12))
                        .build();

        DriveToFirstMark = drive.actionBuilder(new Pose2d(60,-12, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(34.75,-30))
                        .strafeToConstantHeading(new Vector2d(34.75,-48))
                        .strafeToConstantHeading(new Vector2d(60,-12))
                        .build();
        DriveOutofLaunchZone = drive.actionBuilder(new Pose2d(60,-12,Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(20,-12))
                        .build();

                // WAIT for START/PLAY to be pushed
        while(!isStarted()){
            telemetry.update();
        }
        resetRuntime();
        Gericka_Hardware.autoTimeLeft = 0.0;
        //control.SetClawPosition(Gericka_Hardware.CLAW_MAX_POS);


        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(2),
                        DriveToSecondMark,
                        new SleepAction(2),
                        DriveToFirstMark,
                        new SleepAction(2),
                        DriveOutofLaunchZone));

        drive.updatePoseEstimate();

        Gericka_Hardware.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", Gericka_Hardware.autoTimeLeft);
        telemetry.update();

    }
}

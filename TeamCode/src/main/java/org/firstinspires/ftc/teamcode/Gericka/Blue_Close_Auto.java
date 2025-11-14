package org.firstinspires.ftc.teamcode.Gericka;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Gericka.Gericka_MecanumDrive.PARAMS;

import org.firstinspires.ftc.teamcode.PinpointLocalizer;

import java.util.Vector;

public class Blue_Close_Auto extends LinearOpMode {
    Gericka_Hardware control = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;

    public static final String ALLIANCE_KEY = "Alliance";

    // Trajectories
    Action DriveToClosestLine;
    Action DriveClosestLineBackToLaunchPark;

    Action DriveOffTriangleToLaunchSpot;

@Override

    public void runOpMode(){
    //We will start at big trianlge start
    startPose = new Pose2d(-60,-39,Math.toRadians(270));

    drive = new Gericka_MecanumDrive(hardwareMap, startPose);

    //Init hardware as Blue
    control.Init(hardwareMap, "BLUE");
    blackboard.put(ALLIANCE_KEY, "BLUE");


    // Turret initial rough angle toward speaker (tune as needed)
    double turretTargetAngle = -89.0;
    control.SetTurretRotationAngle(turretTargetAngle);

    // Lifter halfway so we can hold 3
    control.SetLifterPosition(control.LIFTER_MID_POSITION);

    telemetry.update();

    DriveToClosestLine = drive.actionBuilder(
            new Pose2d(-60, -39, Math.toRadians(270))).strafeToConstantHeading(new Vector2d(-12, -53)).build();

    DriveClosestLineBackToLaunchPark = drive.actionBuilder(new Pose2d(-12,-53,Math.toRadians(270))).strafeToConstantHeading(new Vector2d(-54, -16)).build();

    DriveOffTriangleToLaunchSpot = drive.actionBuilder((new Pose2d(-60, -39, Math.toRadians(270))))
            .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(270))
            .build();

    // Background telemetry thread (same pattern as Blue_Far_Auto)
    Thread secondaryThread = new Thread(() -> {
        while (!isStopRequested() && getRuntime() < 30) {
            control.ShowPinpointTelemetry();
            telemetry.update();
            sleep(20);
        }
    });
    secondaryThread.start();

    // *************************************
    //      Wait for start
    // *************************************
    while (!isStarted() && !isStopRequested()) {
        telemetry.update();
    }

    resetRuntime();
    Gericka_Hardware.autoTimeLeft = 0.0;

    Actions.runBlocking(DriveOffTriangleToLaunchSpot);

    // Turn turret more directly to target for auto shooting (tune on field)
    turretTargetAngle = -130.0;
    control.SetTurretRotationAngle(turretTargetAngle);

    // Shooter RPM for big triangle shots (tune as needed)
    double shooterSpeedRPM = 3500;
    control.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
    sleep(250);

    control.SetLifterUp();   // shot 1
    sleep(500);
    control.SetLifterDown();
    sleep(500);

    control.SetLifterUp();   // shot 2
    sleep(500);
    control.SetLifterDown();
    sleep(500);

    control.SetLifterUp();   // shot 3
    sleep(500);
    control.SetLifterDown();

    //Shouljd we turn intake on while we go to the closest line
    control.SetIntakeMotor(true, true);


    Actions.runBlocking(
            new SequentialAction(
                    DriveToClosestLine,
                    new SleepAction(0.5),
                    DriveClosestLineBackToLaunchPark
            )
    );

    control.SetIntakeMotor(false, true);


    sleep(500);//giving it sleepy time

    control.SetLifterUp();   // shot 1
    sleep(500);
    control.SetLifterDown();
    sleep(500);

    control.SetLifterUp();   // shot 2
    sleep(500);
    control.SetLifterDown();
    sleep(500);

    control.SetLifterUp();   // shot 3
    sleep(500);
    control.SetLifterDown();

    drive.updatePoseEstimate();

    // turn off shooter wheel
    shooterSpeedRPM = 0.0; //3200rpm was about the value observed when the Motor was commanded to 75%.
    control.SetShooterMotorToSpecificRPM(shooterSpeedRPM);

    // return turret to zero position
    turretTargetAngle = 0.0;
    control.SetTurretRotationAngle(turretTargetAngle);

    Gericka_Hardware.autoTimeLeft = 30-getRuntime();
    telemetry.addData("Time left", Gericka_Hardware.autoTimeLeft);
    telemetry.update();


}


}

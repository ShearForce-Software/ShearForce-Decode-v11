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
@Autonomous(name="Blue Close Auto", preselectTeleOp ="")


public class Blue_Close_Auto extends LinearOpMode {
    Gericka_Hardware control = new Gericka_Hardware(false, false, this);
    Gericka_MecanumDrive drive;
    Pose2d startPose;

    public static final String ALLIANCE_KEY = "Alliance";

    // Trajectories
    Action DriveStartToMidPosition;
    Action DriveMidToClosestLine;
    Action DriveClosestLineBackToLaunchPark;

@Override

    public void runOpMode(){
    //We will start at big trianlge start
    startPose = new Pose2d(-60,-39,Math.toRadians(270));

    drive = new Gericka_MecanumDrive(hardwareMap, startPose);

    //Init hardware as Blue
    control.Init(hardwareMap, "BLUE");
    blackboard.put(ALLIANCE_KEY, "BLUE");


    // Turret initial rough angle toward speaker (tune as needed)
    double turretTargetAngle = 135.8;
    control.SetTurretRotationAngle(turretTargetAngle);

    // Lifter halfway so we can hold 3
    control.SetLifterPosition(control.LIFTER_MID_POSITION);

    telemetry.update();

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

    Actions.runBlocking(new SleepAction((2.0)));
    Actions.runBlocking(DriveStartToMidPosition);

    // Turn turret more directly to target for auto shooting (tune on field)
    turretTargetAngle = 45;    // CHANGE LLATER
    control.SetTurretRotationAngle(turretTargetAngle);

    // Shooter RPM for big triangle shots (tune as needed)
    double shooterSpeedRPM = 2900;
    control.SetShooterMotorToSpecificRPM(shooterSpeedRPM);
    sleep(5000);

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
                    // Mid -> closest line
                    DriveMidToClosestLine,
                    new SleepAction(0.5),           // tiny sleep to finish ingesting rings
                    // Closest line -> launch park
                    DriveClosestLineBackToLaunchPark,
                    // Final 2 second wait to match MeepMeep .waitSeconds(2) at end
                    new SleepAction(2.0)
            )
    );

    control.SetIntakeMotor(false, true);


   // sleep(5000);//giving it sleepy time

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

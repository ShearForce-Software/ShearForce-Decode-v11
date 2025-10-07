package org.firstinspires.ftc.teamcode.Gericka;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Geronimo.Geronimo;
import org.firstinspires.ftc.teamcode.Geronimo.MecanumDrive_Geronimo;


@Autonomous(name="NOT GeronimoAutoRedLeft")
@Disabled
public class AkshayLimeLight extends LinearOpMode {
    Geronimo control = new Geronimo(true, false,this);
    MecanumDrive_Geronimo drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    Pose2d stackPose;
    Action WallTraj;
    Action DriveToStack;
    Action BoxTraj;
    Action Park;
    Action DriveBackToStack;
    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;

    double stackY = 36;
    double stackX = -59;
    double wallDriveY = 58.5;

    double autoPosition = 3;
    Limelight3A limelight;
    VelConstraint velocityConstraint;
    AccelConstraint accelerationConstraint;



    public void runOpMode(){
        startPose = new Pose2d(-12,-60, Math.toRadians(90));
        // stackPose = new Pose2d(stackX, stackY, Math.toRadians(180)); //-54.5,-11.5

        // Define some custom constraints to use when wanting to go faster than defaults
        //speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        //speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        //slowDownVelocityConstraint = new TranslationalVelConstraint(5);
        //slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);

        //velocityConstraint = new TranslationalVelConstraint(30);
        //accelerationConstraint = new ProfileAccelConstraint(15.0, 20.0);

        /* Initialize the Robot */
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap); //Init the hardwareMap
        control.InitLimelight(hardwareMap); // Init the limeLight


        telemetry.update();
        control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());


        waitForStart();
        resetRuntime();
        //control.WebcamInit(hardwareMap);

        while(opModeIsActive()){
            //Check Limelight detection
            if(control.limelightHasTarget()){
                telemetry.addLine("Object Detected");
            }
            else{
                telemetry.addLine("No Object Detected");
            }
            telemetry.update();
        }

        limelight.stop();

    }



}


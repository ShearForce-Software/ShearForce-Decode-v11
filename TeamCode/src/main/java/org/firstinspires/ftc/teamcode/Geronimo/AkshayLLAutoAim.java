package org.firstinspires.ftc.teamcode.Geronimo;

import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.roadrunner.trajectory.TrajectorySequence;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "LL AutoAim", group = "Linear OpMode")
@Disabled
public class AkshayLLAutoAim extends LinearOpMode{
    Geronimo control = new Geronimo(true, false,this);




    private Limelight3A limelight;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
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

    //LLConstants
    private static final double CAMERA_HEIGHT = 10.0; // inches
    private static final double TARGET_HEIGHT = 60.0; // inches
    private static final double CAMERA_ANGLE = 25.0;// degrees
    private static final double DESIRED_DISTANCE = 24.0; //desired stopping distance
    private static final double KpAim = 0.1; //Proportional control constant for aiming
    private static final double KpDrive = 0.05; //Proportional control constant for driving

    //public DcMotorEx leftFront, leftRear, rightRear, rightFront;


    @Override
    public void runOpMode(){
        startPose = new Pose2d(0,0, Math.toRadians(0));
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap); //Init the hardwareMap
        control.InitLimelight(hardwareMap); // Init the limeLight





        waitForStart();

        while(opModeIsActive()){
            ArrayList<Double> currentOffset = new ArrayList<>(control.FindAlignAngleToTargetImage("bottle"));

            if(!currentOffset.isEmpty()){
                double offsetX = currentOffset.get(0);
                double offsetY = currentOffset.get(1);
                System.out.println("Offset X:" + offsetX);
                System.out.println("Offset Y: " + offsetY);
            }
            else{
                System.out.println("NOTHINGUND");
            }

            telemetry.update();
        }

    }

}

package org.firstinspires.ftc.teamcode.Geronimo;
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

import java.util.Vector;


@Autonomous(name="Level1AscentLeft", preselectTeleOp = "Geronimo 1 Manual Control")
@Disabled
public class GeronimoRedLeftAscent extends LinearOpMode {
    Geronimo control = new Geronimo(true, false,this);
    MecanumDrive_Geronimo drive;
    Pose2d startPose;
    Action WallTraj;
    Action SubmersibleTraj;
    Action Park;
    VelConstraint speedUpVelocityConstraint;
    AccelConstraint speedUpAccelerationConstraint;
    VelConstraint slowDownVelocityConstraint;
    AccelConstraint slowDownAccelerationConstraint;

    double wallDriveY = 58.5;


    public void runOpMode(){
        startPose = new Pose2d(-12,-63, Math.toRadians(90));
        // stackPose = new Pose2d(stackX, stackY, Math.toRadians(180)); //-54.5,-11.5

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(5);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);

        /* Initialize the Robot */
        drive = new MecanumDrive_Geronimo(hardwareMap, startPose);
        control.Init(hardwareMap);
        //control.WebcamInit(hardwareMap);
        telemetry.update();
        control.imuOffsetInDegrees = 270; // Math.toDegrees(startPose.heading.toDouble());
        control.SetClawPosition(Geronimo.CLAW_MAX_POS);
        sleep(500);
        control.AutoStartPosition();

        while(!isStarted()){
            telemetry.update();
        }
        resetRuntime();
        control.autoTimeLeft = 0.0;

        // ***************************************************
        // ****  START DRIVING    ****************************
        // ***************************************************

        SubmersibleTraj= drive.actionBuilder(drive.localizer.getPose())
                // .strafeToLinearHeading(new Vector2d(36,  36), Math.toRadians(-90))
                //   .strafeToLinearHeading(new Vector2d(36, 12), Math.toRadians((-90)))
                //  .strafeToLinearHeading(new Vector2d(28,6), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-48,-11, Math.toRadians(0)), Math.toRadians(90))
                .strafeTo(new Vector2d(-28, -11))
                .build();


      /*  WallTraj = drive.actionBuilder(new Pose2d(-12,-30,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-12, -36), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                .build();

       */



        /* Drive to the Board */
        Actions.runBlocking(
                new ParallelAction(
                        SubmersibleTraj,
                        new SequentialAction(
                                specimenDeliverLow()
                        )
                )


        );


        drive.updatePoseEstimate();

        // Build up the Stack to Submerssible Trajectory



        // drive.updatePoseEstimate();
        // Build up the Stack to Wall Trajectory




        /*
        Park = drive.actionBuilder(drive.pose)
                .lineToX(45, slowDownVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(48, 56), Math.toRadians(270))
                .build();
        Actions.runBlocking(
                new ParallelAction(
                        Park
                )
        );

         */

        control.autoTimeLeft = 30-getRuntime();
        telemetry.addData("Time left", control.autoTimeLeft);
        telemetry.update();

    }
    public Action specimenDeliverLow (){return new SpecimenDeliverLow();}
    public class SpecimenDeliverLow implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.BasketHigh();
                initialized = true;
            }
            packet.put("SpecimenDeliverLow", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

    public Action grabsample (){return new GrabSample();}
    public class GrabSample implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {

                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }

    }
}


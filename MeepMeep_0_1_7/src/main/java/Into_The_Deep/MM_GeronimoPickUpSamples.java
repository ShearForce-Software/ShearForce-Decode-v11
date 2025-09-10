package Into_The_Deep;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MM_GeronimoPickUpSamples {

    static Action CornerTraj;
    static Action SubmersibleTraj;
    static Action CornerTraj2;
    static Action SampleTraj;
    static Action CornerTraj3;
    static Action StrafeTraj;
    static Action CornerTraj4;
    static Action Park;

    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;
    static VelConstraint slowDownVelocityConstraint;
    static AccelConstraint slowDownAccelerationConstraint;
    static VelConstraint intakeVelocityConstraint;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d StartPose = new Pose2d(-12, -63, Math.toRadians(90));

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);
        intakeVelocityConstraint = new TranslationalVelConstraint(15);

        SubmersibleTraj= myBot.getDrive().actionBuilder(StartPose)
                .strafeToLinearHeading(new Vector2d(-12,-36), Math.toRadians(90))
                //Deliver specimen to submersible
                .build();


        CornerTraj = myBot.getDrive().actionBuilder(new Pose2d(-12,-36,Math.toRadians(90)))
                .strafeTo(new Vector2d(-12,-40))
                .splineToLinearHeading(new Pose2d(-48,-29, Math.toRadians(180)), Math.toRadians(180))
                //Intake the sample
                .build();

        CornerTraj2 = myBot.getDrive().actionBuilder(new Pose2d(-48, -29, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                //outtake the sample into the corner
                .build();

        SampleTraj = myBot.getDrive().actionBuilder(new Pose2d(-48, -48, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(-53, -29), Math.toRadians(90))
                //intake the sample on the floor
                .build();

        CornerTraj3 = myBot.getDrive().actionBuilder(new Pose2d(-53,-29, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                //outtake sample
                .build();

        StrafeTraj = myBot.getDrive().actionBuilder(new Pose2d(-48, -48, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(-60,-48), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-62, -48), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-62, -29), Math.toRadians(90))
                // intake specimen on ground
                .build();

        CornerTraj4 = myBot.getDrive().actionBuilder(new Pose2d(-62, -29, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-55, -29), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(225))
                //outtake
                .build();



        myBot.runAction(
                new SequentialAction(
                        SubmersibleTraj,
                        CornerTraj,
                        CornerTraj2,
                        SampleTraj,
                        CornerTraj3,
                        StrafeTraj,
                        CornerTraj4

                )

                //in the parallel action
                        /*  new SequentialAction(
                                specimenDeliverLow()
                        )

                         */


        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

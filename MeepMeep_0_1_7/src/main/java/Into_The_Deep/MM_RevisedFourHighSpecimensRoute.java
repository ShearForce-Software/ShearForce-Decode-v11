package Into_The_Deep;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MM_RevisedFourHighSpecimensRoute {
    static Action DeliverStartingSpecimen;
    static Action DriveToSample1;
    static Action DeliverSample1;
    static Action DriveToSample2;
    static Action DeliverSample2;
    static Action DrivetoDeck1;
    static Action DriveToSubmersible1;
    static Action DrivetoDeck2;
    static Action DriveToSubmersible2;
    static Action DrivetoDeck3;
    static Action DriveToSubmersible3;
    static Action ParkinDeck;

    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;
    static VelConstraint normalVelocityConstraint;
    static AccelConstraint normalAccelerationConstraint;
    static VelConstraint slowDownVelocityConstraint;
    static AccelConstraint slowDownAccelerationConstraint;
    static VelConstraint intakeVelocityConstraint;
    static VelConstraint humanPlayerVelocityConstraint;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

       // static Pose2d startPose;
    // Testing from home


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, Math.PI*.6, Math.PI*.8, 15)
                .build();



        Pose2d StartPose = new Pose2d(9, -64, Math.toRadians(270));
// Fix it
        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(60.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-40.0, 60.0);
        normalVelocityConstraint = new TranslationalVelConstraint(50.0);
        normalAccelerationConstraint = new ProfileAccelConstraint(-35.0, 50.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(30);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-20, 50);
        intakeVelocityConstraint = new TranslationalVelConstraint(15);
        humanPlayerVelocityConstraint = new TranslationalVelConstraint(7);

        //myBot.runAction(myBot.getDrive().actionBuilder(StartPose)
                           //  .splineToLinearHeading(new Pose2d(36,48,36.6), Math.toRadians(270))
        DeliverStartingSpecimen = myBot.getDrive().actionBuilder(StartPose)
                //.strafeToLinearHeading(new Vector2d(4,-30), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(0,-30), Math.toRadians(270), slowDownVelocityConstraint)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(9,-39), Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(9,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DriveToSample1 = myBot.getDrive().actionBuilder(new Pose2d(9, -30, Math.toRadians(270)))
                //.strafeToLinearHeading(new Vector2d(0,-48), Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(36,-48,Math.toRadians(270)), Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                //.splineToConstantHeading(new Vector2d(36,-48), Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(36,-48,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .splineTo(new Vector2d(48, -45),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                //.strafeToLinearHeading(new Vector2d(44,-54),Math.toRadians(270))
                .build();
        DeliverSample1 = myBot.getDrive().actionBuilder(new Pose2d(48,-45, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(57,-45), Math.toRadians(270))
                .build();
        DriveToSample2 = myBot.getDrive().actionBuilder(new Pose2d(57,-45, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(58,-45), Math.toRadians(90))
                .build();
        DeliverSample2 = myBot.getDrive().actionBuilder(new Pose2d(58,-45, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(48,-45), Math.toRadians(270))
                .build();
        DrivetoDeck1 = myBot.getDrive().actionBuilder(new Pose2d(48,-45,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(48,-54,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(48, -63),Math.toRadians(270), humanPlayerVelocityConstraint)
                .build();

        DriveToSubmersible1 = myBot.getDrive().actionBuilder(new Pose2d(48,-63,Math.toRadians(270)))
                .setReversed(true)
                //.strafeToLinearHeading(new Vector2d(48, -54), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(2, -54), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(16,-56), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(2,-39),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(2,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        DrivetoDeck2 = myBot.getDrive().actionBuilder(new Pose2d(2,-30,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(38,-54,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(38,-63), Math.toRadians(270), humanPlayerVelocityConstraint) //May change to 270 heading once delivery is clarified
                .build();
        DriveToSubmersible2 = myBot.getDrive().actionBuilder(new Pose2d(38,-63,Math.toRadians(270)))
                .setReversed(true)
                //.strafeTkjoLinearHeading(new Vector2d(16,-56), Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(4,-39,Math.toRadians(270)), Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .splineToConstantHeading(new Vector2d(3,-39),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(3,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();
        DrivetoDeck3 = myBot.getDrive().actionBuilder(new Pose2d(3,-30,Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(38,-54,Math.toRadians(270)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(38,-63), Math.toRadians(270), humanPlayerVelocityConstraint) //May change to 270 heading once delivery is clarified
                .build();
        DriveToSubmersible3 = myBot.getDrive().actionBuilder(new Pose2d(38,-63,Math.toRadians(270)))
                .setReversed(true)
                //.strafeToLinearHeading(new Vector2d(16,-56), Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(3,-39,Math.toRadians(270)), Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .splineToConstantHeading(new Vector2d(4,-39),Math.toRadians(90), normalVelocityConstraint, normalAccelerationConstraint)
                .strafeToLinearHeading(new Vector2d(4,-30), Math.toRadians(270), intakeVelocityConstraint)
                .build();

        /*ParkinDeck = myBot.getDrive().actionBuilder(new Pose2d(4,-30,Math.toRadians(270)))
                //Pose 2D 50,-54, 270
                //.splineTo(new Vector2d(49,-45),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(4,-35), Math.toRadians(270), intakeVelocityConstraint)
                //.strafeToLinearHeading(new Vector2d(36,-58), Math.toRadians(90), intakeVelocityConstraint)
                //.splineToLinearHeading(new Pose2d(30,-48,Math.toRadians(90)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                .splineToLinearHeading(new Pose2d(38,-58,Math.toRadians(90)),Math.toRadians(270), normalVelocityConstraint, normalAccelerationConstraint)
                //.strafeToLinearHeading(new Vector2d(2,-50), Math.toRadians(270))
                //.strafeToLinearHeading(new Vector2d(49,-58), Math.toRadians(90), speedUpVelocityConstraint)
                // .turnTo(Math.toRadians(90))
                .build();
                DriveToSamplesandDeliver3 = myBot.getDrive().actionBuilder(new Pose2d(56,-54,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(54, -12), Math.toRadians(270), slowDownVelocityConstraint)
                .strafeToLinearHeading(new Vector2d(60.75,-12), Math.toRadians(270))
                //.splineToConstantHeading(new Vector2d(63,-17), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(60.75,-54), Math.toRadians(270))
                //.lineToYConstantHeading(-54)
                //.strafeToLinearHeading(new Vector2d(48,-54),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(44,-46),Math.toRadians(270))
                .build();
*/



        myBot.runAction(new SequentialAction(
                //Drive to submersible and pick up sample
                DeliverStartingSpecimen,
                new SleepAction(1.5),
                //Extend slides and prepare to grab sample
                DriveToSample1,
                new SleepAction(.5),
                DeliverSample1,
                new SleepAction(.5),
                DriveToSample2,
                new SleepAction(.5),
                DeliverSample2,
                DrivetoDeck1,
                DriveToSubmersible1,
                DrivetoDeck2,
                DriveToSubmersible2,
                DrivetoDeck3,
                DriveToSubmersible3


                ));

                //new SleepAction(1)
                //Drop submersible
                //DropOff1,
                //new SleepAction(1),
                //drive to submersible and pickup sample
               /* DriveToSubmersible2,
                new SleepAction(1),
                //drop sample and pick up specimen
                DropOff2,
                new SleepAction(1),
                PickSpeicmen,
                new SleepAction(1),
                //drop specimen and pick up sample
                SpecimenDrop//,
               */ //new SleepAction(1);



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

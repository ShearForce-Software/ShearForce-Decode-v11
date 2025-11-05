package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    /** Dashboard-tunable params */
    public static class Params {
        // IMU orientation ( Pinpoint handles heading internally)
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        // Drive model parameters
        public double inPerTick = 0.000763844685;
        public double lateralInPerTick = 0.0005929748631909519;
        public double trackWidthTicks = 15976.273158890097;

        // Feedforward (tick units)
        public double kS = 0.680200321870422;
        public double kV = 0.0001411843245357985;
        public double kA = 0.00003;

        // Path profile (in)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // Turn profile (rad)
        public double maxAngVel = Math.PI;
        public double maxAngAccel = Math.PI;

        // Holonomic controller gains (nonzero defaults)
        public double axialGain = 5.0;
        public double lateralGain = 5.0;
        public double headingGain = 10.0;

        public double axialVelGain = 0.0;
        public double lateralVelGain = 1.0;
        public double headingVelGain = 0.5;
    }

    public static double imuOffsetRadians = 0.0; // retained for compatibility
    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);

    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));

    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public final VoltageSensor voltageSensor;

    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    /** Optional internal wheel+REV IMU localizer kept for reference/testing */
    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack  = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront= new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));
            this.pose = pose;

            // Wheel encoder sign (typical RR mecanum)
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        @Override public void setPose(Pose2d pose) { this.pose = pose; }
        @Override public Pose2d getPose() { return pose; }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair lf = leftFront.getPositionAndVelocity();
            PositionVelocityPair lb = leftBack.getPositionAndVelocity();
            PositionVelocityPair rb = rightBack.getPositionAndVelocity();
            PositionVelocityPair rf = rightFront.getPositionAndVelocity();

            // If you wire this localizer back in, supply yaw yourself (REV IMU)
            YawPitchRollAngles dummy = new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0);
            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS",
                    new MecanumLocalizerInputsMessage(lf, lb, rb, rf, dummy));

            Rotation2d heading = Rotation2d.exp(0.0);
            if (!initialized) {
                initialized = true;
                lastLeftFrontPos = lf.position;
                lastLeftBackPos  = lb.position;
                lastRightBackPos = rb.position;
                lastRightFrontPos= rf.position;
                lastHeading = heading;
                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = kinematics.forward(
                    new MecanumKinematics.WheelIncrements<>(
                            new DualNum<Time>(new double[]{
                                    (lf.position - lastLeftFrontPos), lf.velocity
                            }).times(PARAMS.inPerTick),
                            new DualNum<Time>(new double[]{
                                    (lb.position - lastLeftBackPos),  lb.velocity
                            }).times(PARAMS.inPerTick),
                            new DualNum<Time>(new double[]{
                                    (rb.position - lastRightBackPos), rb.velocity
                            }).times(PARAMS.inPerTick),
                            new DualNum<Time>(new double[]{
                                    (rf.position - lastRightFrontPos), rf.velocity
                            }).times(PARAMS.inPerTick)
                    )
            );


            lastLeftFrontPos = lf.position;
            lastLeftBackPos  = lb.position;
            lastRightBackPos = rb.position;
            lastRightFrontPos= rf.position;
            lastHeading = heading;

            pose = pose.plus(new Twist2d(twist.line.value(), headingDelta));
            return twist.velocity().value();
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d initialPose) {
        imuOffsetRadians = initialPose.heading.toDouble();

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Motor names must match your FTC config
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront_leftOdometry");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear_rightOdometry");
        rightFront= hardwareMap.get(DcMotorEx.class, "rightFront_centerOdometry");

        // Standard mecanum directions (verify on your bot)
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // *** USE PINPOINT akshayyy***
        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, initialPose);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels =
                new MecanumKinematics(1).inverse(PoseVelocity2dDual.constant(powers, 1));

        double maxMag = 1.0;
        for (DualNum<Time> w : wheelVels.all()) maxMag = Math.max(maxMag, Math.abs(w.value()));

        leftFront.setPower(wheelVels.leftFront.get(0) / maxMag);
        leftBack.setPower (wheelVels.leftBack.get(0)  / maxMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxMag);
        rightFront.setPower(wheelVels.rightFront.get(0)/ maxMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;
        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;
            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(), Math.max(2, (int) Math.ceil(t.path.length() / 2)));

            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) { beginTs = Actions.now(); t = 0; }
            else { t = Actions.now() - beginTs; }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0); leftBack.setPower(0);
                rightBack.setPower(0); rightFront.setPower(0);
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            double voltage = Math.max(1e-6, voltageSensor.getVoltage());
            MotorFeedforward ff = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            double lf = ff.compute(wheelVels.leftFront) / voltage;
            double lb = ff.compute(wheelVels.leftBack)  / voltage;
            double rb = ff.compute(wheelVels.rightBack) / voltage;
            double rf = ff.compute(wheelVels.rightFront)/ voltage;

            mecanumCommandWriter.write(new MecanumCommandMessage(voltage, lf, lb, rb, rf));

            leftFront.setPower(lf); leftBack.setPower(lb);
            rightBack.setPower(rb); rightFront.setPower(rf);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beginTs = -1;

        public TurnAction(TimeTurn turn) { this.turn = turn; }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) { beginTs = Actions.now(); t = 0; }
            else { t = Actions.now() - beginTs; }

            if (t >= turn.duration) {
                leftFront.setPower(0); leftBack.setPower(0);
                rightBack.setPower(0); rightFront.setPower(0);
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            double voltage = Math.max(1e-6, voltageSensor.getVoltage());
            MotorFeedforward ff = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

            double lf = ff.compute(wheelVels.leftFront) / voltage;
            double lb = ff.compute(wheelVels.leftBack)  / voltage;
            double rb = ff.compute(wheelVels.rightBack) / voltage;
            double rf = ff.compute(wheelVels.rightFront)/ voltage;

            mecanumCommandWriter.write(new MecanumCommandMessage(voltage, lf, lb, rb, rf));

            leftFront.setPower(lf); leftBack.setPower(lb);
            rightBack.setPower(rb); rightFront.setPower(rf);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        while (poseHistory.size() > 100) poseHistory.removeFirst();
        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xs = new double[poseHistory.size()];
        double[] ys = new double[poseHistory.size()];
        int i = 0;
        for (Pose2d t : poseHistory) {
            xs[i] = t.position.x;
            ys[i] = t.position.y;
            i++;
        }
        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xs, ys);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(1e-6, new ProfileParams(0.25, 0.1, 1e-2)),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}

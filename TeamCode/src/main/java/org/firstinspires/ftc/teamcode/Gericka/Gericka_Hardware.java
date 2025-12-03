package org.firstinspires.ftc.teamcode.Gericka;

import static org.firstinspires.ftc.teamcode.Gericka.Gericka_MecanumDrive.PARAMS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.PinpointLocalizer;
//import org.firstinspires.ftc.teamcode.testign123;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Config
public class Gericka_Hardware {
    //ReentrantLock lock = new ReentrantLock();
    public static final String ALLIANCE_KEY = "Alliance";
    public static final String FINAL_X_POSITION = "Final_X_Position";
    public static final String FINAL_Y_POSITION = "Final_Y_Position";
    public static final String FINAL_HEADING_DEGREES = "Final_Heading_Degrees";

    IMU imu;
    public double imuOffsetInDegrees = 0.0;
    double imuPosition = 0;

    Gericka_MecanumDrive drive;

    LinearOpMode opMode;
    public static double autoTimeLeft = 0.0;
    boolean IsDriverControl;
    boolean IsFieldCentric;

    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightFront;
    DcMotorEx rightRear;

    DcMotorEx intakeMotor;
    DcMotorEx shooterMotorRight;
    DcMotorEx shooterMotorLeft;
    DcMotorEx turretMotor;
    Servo lifterServo;
    Servo light1;
    Servo light2;
    Servo light3;

    //pidf rotator variables
    public static boolean pidfEnabled = false;
    public static double p = 0.004, i = 0, d = 0, f = 0.007; //0.0001 > p was .005
    PIDController Right_controller = new PIDController(p, i, d);
    PIDController Left_controller = new PIDController(p, i, d);

    public static double Kp = 0.0035, Ki = 0, Kd = 0.00021, Kf = 0;
    public static double Ktolerance = 5.0;
    PIDController RightSlide_controller = new PIDController(Kp, Ki, Kd);
    PIDController LeftSlide_controller = new PIDController(Kp, Ki, Kd);

    public double finalX;
    public double finalY;

    //Webcam
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    final double INTAKE_POWER = 0.6
            ;

    public final float LIFTER_UP_POSITION = 0.85f;
    public final float LIFTER_MID_POSITION = 0.5f;
    public final float LIFTER_DOWN_POSITION = 0.05f;

    private boolean useOnlyWebcamForDistance = true;
    private boolean autoTurretMode = true;
    private boolean autoLifterMode = false;
    private boolean autoIntakeMode = false;
    private boolean autoShooterMode = false;
    private boolean useRoadrunnerForTurretAnglesEnabled = true;

    final float MAX_SHOOTER_SPEED = 1.0f;
    final float MIN_SHOOTER_SPEED = 0.0f;
    double shooterTargetRPM = 0.0;
    public static boolean shooterPIDF_Enabled = false;
    public static double shooterP = 0.0;
    public static double shooterI = 0.0;
    public static double shooterD = 0.0;
    public static double shooterF = 0.0;
    final float MAX_SHOOTER_RPM = 4500;
    final float MIN_SHOOTER_RPM = 0;
    final float MAX_TURRET_ANGLE = 122;
    final float MIN_TURRET_ANGLE = -179;
    final double YELLOW_JACKET_19_1_TICKS = 537.7; // 19.2:1 - ticks per motor shaft revolution
    final double YELLOW_JACKET_1_1_TICKS = 28.0; // 1:1 - ticks per motor shaft revolution
    final double YELLOW_JACKET_3_1_TICKS = 103.8; // 3.7:1 - ticks per motor shaft revolution
    final double YELLOW_JACKET_13_1_TICKS = 384.5; // 13.7:1 - ticks per motor shaft revolution
    final double YELLOW_JACKET_5_1_TICKS = 145.1; // 5.2:1 - ticks per motor shaft revolution
    final double TURRET_TICKS_IN_DEGREES = (133.0/24.0/360.0) * YELLOW_JACKET_5_1_TICKS; // 133/24 is the gear ratio
    int turretTargetTicks = 0;
    double turretTargetAngle = 0.0;
    double lifterTargetPosition = 0.0;
    double shooterTargetSpeed = 0.0;
    //public Servo allianceIndicatorLight = null;
    private String allianceColorString = "UNKNOWN";
    final double INDICATOR_BLACK = 0;
    final double INDICATOR_RED = 0.279;
    final double INDICATOR_BLUE = 0.611;
    final double INDICATOR_WHITE = 1;
    final double INDICATOR_GREEN = 0.500;
    final double INDICATOR_ORANGE = 0.333;
    final double INDICATOR_YELLOW = 0.388;
    final double INDICATOR_SAGE_GREEN = 0.444;
    final double INDICATOR_VIOLET = 0.722;
    double indicatorLightValue = 0;
    public final double FEET_TO_METER = 0.3048;
    public final double METER_TO_FEET = 3.28084;
    public final double RADIANS_PER_SECOND_TO_RPM = 9.54929658551; // 60 / (2 * Math.PI)
    public double targetX = 0;
    public double targetY = 0;
    public int currentAprilTargetId = 20;
    GoBildaPinpointDriver pinpoint;
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0,AngleUnit.DEGREES,0);

    RevBlinkinLedDriver.BlinkinPattern Blinken_pattern;
    RevBlinkinLedDriver blinkinLedDriver;
    RevColorSensorV3 ColorSensorRight;
    RevColorSensorV3 ColorSensorLeft;
    //private NormalizedColorSensor ColorSensorRight;
    //private NormalizedColorSensor ColorSensorLeft;

    //ModernRoboticsAnalogTouchSensor beamBreak1;
    TouchSensor beamBreak1;

    //RevColorSensorV3 leftColorSensor;
    //RevColorSensorV3 rightColorSensor;
    //int redLeft = 0;
    //int greenLeft = 0;
    //int blueLeft = 0;
    //int redRight = 0;
    //int greenRight = 0;
    //int blueRight = 0;
    //private int position;
/*
    // REV v3 color sensor variables
    public enum colorEnum {
        noColor,
        red,
        //yellow,
        blue
    }

 */

    /*
    //colorEnum colorDetected = colorEnum.noColor;
    //NAV TO TAG VARIABLES
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    //private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    double rangeError = 0;
    double yawError = 0;
    //double LimelightMountingHeight = 6;  //Adjust when robot is built
    //double LimelightMountingAngle = Math.toDegrees(90);
    //double distance_to_object = 0;
    //double objectHeight = 0;
    //double XDistance_to_object = 0;
    //double YDistance_to_object = 0;
    //double angletoObject = Math.toRadians(60);

     */

    public Gericka_Hardware(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
        this.opMode = opMode;
    }
    public void Init (HardwareMap hardwareMap, String allianceColor) {
        //allianceIndicatorLight = hardwareMap.get(Servo.class, "IndicatorLight");
        // ************* Drive MOTORS ****************
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront_leftOdometry");
        leftRear= hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear_rightOdometry");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront_centerOdometry");

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // GoBilda Pinpoint Odometry computer (reports IMU heading and movement ticks)
        InitPinpoint(hardwareMap);



        // ********** Intake and Shooter System Motors **********************************
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotorLeft");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotorRight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterMotorRight.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setTargetPosition(0);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setPower(0.0);
        shooterMotorLeft.setPower(0.0);
        shooterMotorRight.setPower(0.0);
        turretMotor.setPower(0.0);

        if (GetShooterPIDF_Enabled()) {
            shooterMotorLeft.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            shooterMotorRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        }



            // ****************** SERVOS ******************************************
        lifterServo = hardwareMap.get(Servo.class, "lifterServo");
        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");
        light3 = hardwareMap.get(Servo.class, "light3");

        // ********** Color Sensors ********************
        ColorSensorRight = hardwareMap.get(RevColorSensorV3.class, "ColorSensorRight");
        ColorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");

        //leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");
        //rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorRight");
        //leftColorSensor.enableLed(false);
        //rightColorSensor.enableLed(false);
        if (Objects.equals(allianceColor, "RED")) {
            allianceColorString = "RED";
            //allianceIndicatorLight.setPosition(INDICATOR_RED);
        } else {
            allianceColorString = "BLUE";
            //allianceIndicatorLight.setPosition(INDICATOR_BLUE);
        }

         //limelightbox = hardwareMap.get(Limelight3A.class, "limelight");
        //InitBlinkin(hardwareMap);

        // ***** breakbeam *****
        beamBreak1 =  hardwareMap.get(TouchSensor.class, "beamBreak1");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        imu.initialize(parameters);
        imu.resetYaw();

    }

    public void SetAllianceColor(String value) {
        allianceColorString = value;
    }

    public void ShowTelemetry(){

        ShowPinpointTelemetry();

        ShowRoadrunnerPosition();

        opMode.telemetry.addData("Shooter ", "L-RPM: %.1f, R-RPM: %.1f", CalculateMotorRPM(shooterMotorLeft.getVelocity(), YELLOW_JACKET_1_1_TICKS), CalculateMotorRPM(shooterMotorRight.getVelocity(), YELLOW_JACKET_1_1_TICKS));
        opMode.telemetry.addData("        ", "L-Vel: %.3f, R-Vel: %.3f" , shooterMotorLeft.getVelocity(), shooterMotorRight.getVelocity());
        opMode.telemetry.addData("        ", "L-Pow: %.3f, R-Pow: %.3f" , shooterMotorLeft.getPower(), shooterMotorRight.getPower());
        //opMode.telemetry.addData("        ", "L-Amp: %.1f, R-Amp: %.1f" , shooterMotorLeft.getCurrent(CurrentUnit.AMPS), shooterMotorRight.getCurrent(CurrentUnit.AMPS));
        //opMode.telemetry.addData("Shooter Target Speed: ", shooterTargetSpeed);
        opMode.telemetry.addData("Shooter Target RPM: ", shooterTargetRPM);
        ShowPIDF_Telemetry();

        opMode.telemetry.addData("Auto Shooter Mode: ", autoShooterMode);
        opMode.telemetry.addData("Auto Turret Mode: ", autoTurretMode);
        opMode.telemetry.addData("Auto Lifter Mode: ", autoLifterMode);
        opMode.telemetry.addData("Auto Intake Mode: ", autoIntakeMode);
        opMode.telemetry.addData("Use Only Webcam for Distance: ", GetUseOnlyWebcamForDistance());
        opMode.telemetry.addData("Use pinpoint for turret angles: ", GetUseRoadrunnerForTurretAnglesEnabled());

        opMode.telemetry.addData("April Tag Target ID", currentAprilTargetId);

        opMode.telemetry.addData("Turret ", "ticks: %d, tgt-Angle: %.1f", turretMotor.getCurrentPosition(),turretTargetAngle);
        //opMode.telemetry.addData("       ", "Pow: %.1f, Amp: %.1f", turretMotor.getPower(),turretMotor.getCurrent(CurrentUnit.AMPS));

        opMode.telemetry.addData("Intake ", "Pow: %.1f, Amp: %.1f", intakeMotor.getPower(), intakeMotor.getCurrent(CurrentUnit.AMPS));
        //opMode.telemetry.addData("       ", "Vel: %.1f", intakeMotor.getVelocity());

        opMode.telemetry.addData("Lifter Position: ", lifterServo.getPosition());

        opMode.telemetry.addData("Light Detected", ((OpticalDistanceSensor) ColorSensorLeft).getLightDetected());
        //NormalizedRGBA colorsLeft = ColorSensorLeft.getNormalizedColors();

        opMode.telemetry.addData("Light Detected", ((OpticalDistanceSensor) ColorSensorRight).getLightDetected());
        //NormalizedRGBA colorsRight = ColorSensorRight.getNormalizedColors();
        opMode.telemetry.addData("Light1", light1.getPosition());
        opMode.telemetry.addData("Light2", light2.getPosition());
        opMode.telemetry.addData("Light3", light3.getPosition());

        opMode.telemetry.addData("imu Heading: ", GetIMU_HeadingInDegrees());
        //opMode.telemetry.addData("imu roll: ", (imu.getRobotYawPitchRollAngles().getRoll()));
        //opMode.telemetry.addData("imu pitch: ", (imu.getRobotYawPitchRollAngles().getPitch()));
        //opMode.telemetry.addData("imu yaw: ", (imu.getRobotYawPitchRollAngles().getYaw()));

        opMode.telemetry.addData("Alliance: ", allianceColorString);

        telemetryAprilTag();

        //opMode.telemetry.addData("Auto Last Time Left: ", autoTimeLeft);

        opMode.telemetry.update();
    }

    public void ShowPIDF_Telemetry() {
        PIDFCoefficients shooterPIDF_Left = shooterMotorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.telemetry.addData("ShooterL PIDF: ", "p: %.3f  i: %.2f  d: %.2f  f: %.2f", shooterPIDF_Left.p, shooterPIDF_Left.i, shooterPIDF_Left.d, shooterPIDF_Left.f);

        PIDFCoefficients shooterPIDF_Right = shooterMotorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        opMode.telemetry.addData("ShooterR PIDF: ", "p: %.3f  i: %.2f  d: %.2f  f: %.2f", shooterPIDF_Right.p, shooterPIDF_Right.i, shooterPIDF_Right.d, shooterPIDF_Right.f);

    }

    public void SetShooterPIDFCoefficients() {
        if (GetShooterPIDF_Enabled()) {
            PIDFCoefficients shooterPIDF = shooterMotorLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            // if the static PIDF coefficients changed (probably through the dashboard)
            if ((shooterPIDF.p != shooterP) || (shooterPIDF.i != shooterI) || (shooterPIDF.d != shooterD) || (shooterPIDF.f != shooterF)) {
                shooterMotorLeft.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
                shooterMotorRight.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
            }
        }
    }

    public boolean GetShooterPIDF_Enabled() { return shooterPIDF_Enabled; }
    public void SetShooterPIDF_Enabled(boolean value) { shooterPIDF_Enabled = value; }

    public void EndgameBuzzer(){
        if(opMode.getRuntime() < 84.5 && opMode.getRuntime() > 84.0){
            opMode.gamepad1.rumble(1000);
            opMode.gamepad2.rumble(1000);
        }
    }
    public void InitRoadRunner(Gericka_MecanumDrive roadrunner)
    {
        drive = roadrunner;
    }

    public void ShowRoadrunnerPosition() {
        if (drive != null) {
            drive.updatePoseEstimate();
            opMode.telemetry.addData("Roadrunner Position(inches): ", "x: %.1f, y: %.1f", drive.localizer.getPose().position.x, drive.localizer.getPose().position.y);
            opMode.telemetry.addData("Roadrunner Heading(deg): ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        }
    }
    public void SetRoadrunnerPosition(double xPositionInches, double yPositionInches, double headingDegrees){
        startPose = new Pose2D(DistanceUnit.INCH, xPositionInches, yPositionInches, AngleUnit.DEGREES, headingDegrees);
    }
    public Pose2D GetRoadrunnerAbsolutePosition(Pose2D pos){
        double AbsoluteX = drive.localizer.getPose().position.x + pos.getX(DistanceUnit.INCH);
        double AbsoluteY = drive.localizer.getPose().position.y + pos.getY(DistanceUnit.INCH);
        double AbsoluteAngle = Math.toDegrees(drive.localizer.getPose().heading.toDouble()) + pos.getHeading(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, AbsoluteX, AbsoluteY,AngleUnit.DEGREES, AbsoluteAngle);
    }


    // *************************************************************************
    //      Pinpoint Utility Functions
    // *************************************************************************

    public void InitPinpoint(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        // Xpod is -84.0 MM from pinpoint computer, and -109.5 mm from center point of robot
        // Ypod is aligned with pinpoint computer in X, and -198.0 mm behind the center point of robot
        double xOffset = -109.5; //mm - X pod is to the right of pinpoint and center when viewed from the top
        double yOffset = -198.0; // mm - Y pod is -198 mm behind the center rotation point of chassis
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        // need to verify X,Y directions, going forward should make X count up, going Left should make Y count up
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD
                , GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
        //pinpoint.recalibrateIMU();

        pinpoint.update();
    }

    public void ShowPinpointTelemetry() {
        if (pinpoint != null) {
            pinpoint.update();

        /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
        */
            opMode.telemetry.addData("pinpoint status: ", pinpoint.getDeviceStatus());
            opMode.telemetry.addData("pinpoint ", "x-ticks: %d, y-ticks: %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
            opMode.telemetry.addData("pinpoint Relative Pos(inches): ", "x: %.1f, y: %.1f", pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
            opMode.telemetry.addData("pinpoint Relative Position(inches): ", "x: %.1f, y: %.1f", pinpoint.getPosition().getX(DistanceUnit.INCH), pinpoint.getPosition().getY(DistanceUnit.INCH));
            opMode.telemetry.addData("pinpoint Relative Heading(deg): ", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));

            Pose2D currentPosition = GetPinpointAbsolutePosition(startPose);
            opMode.telemetry.addData("pinpoint Absolute Position(inches): ", "x: %.1f, y: %.1f", currentPosition.getX(DistanceUnit.INCH), currentPosition.getY(DistanceUnit.INCH));
            opMode.telemetry.addData("pinpoint Absolute Heading(deg): ", currentPosition.getHeading(AngleUnit.DEGREES));
        }

    }
    public void SetInitalPinpointPosition(double xPositionInches, double yPositionInches, double headingDegrees) {
        //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xPositionInches, yPositionInches, AngleUnit.DEGREES, headingDegrees));
        startPose = new Pose2D(DistanceUnit.INCH, xPositionInches, yPositionInches, AngleUnit.DEGREES, headingDegrees);
        //pinpoint.update();
    }
    public void setPinpointPositionFromWebcam() {
        double currentX = 0;
        double currentY = 0;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // only use the two goals, the obelisk could be at a variable location
                if ((detection.id == 20) || (detection.id == 24)) {
                    currentX = detection.ftcPose.x;
                    currentY = detection.ftcPose.y;
                    if (!leftFront.isBusy() && !leftRear.isBusy() && !rightFront.isBusy() && !rightRear.isBusy()) {
                        //if (pinpoint.getHeadingVelocity() = 0){
                        SetInitalPinpointPosition(currentX, currentY, pinpoint.getHeading(AngleUnit.DEGREES));
                        break;
                    }
                }
            }
        }
        pinpoint.update();

    }
    public Pose2D GetPinpointAbsolutePosition(Pose2D pos){
        double AbsoluteX = pinpoint.getPosY(DistanceUnit.INCH) + pos.getX(DistanceUnit.INCH);
        double AbsoluteY = pinpoint.getPosX(DistanceUnit.INCH) + pos.getY(DistanceUnit.INCH);
        double AbsoluteAngle = pinpoint.getHeading(AngleUnit.DEGREES) - pos.getHeading(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, AbsoluteX, AbsoluteY,AngleUnit.DEGREES, AbsoluteAngle);
    }

    // *************************************************************************
    //      Webcam & April Tag Utility Functions
    // *************************************************************************

   public void WebcamInit (HardwareMap hardwareMap){
       aprilTag = new AprilTagProcessor.Builder().build();

       VisionPortal.Builder builder = new VisionPortal.Builder();
       builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
       builder.addProcessor(aprilTag);

       visionPortal = builder.build();
    }
    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        opMode.telemetry.addData("Webcam Tags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                opMode.telemetry.addLine(String.format("Webcam Tag ID %d (%s)", detection.id, detection.metadata.name));
                opMode.telemetry.addData("Range", "%.1f in", detection.ftcPose.range);
                opMode.telemetry.addData("Bearing", "%.1f°", detection.ftcPose.bearing);
                opMode.telemetry.addData("Elevation", "%.1f°", detection.ftcPose.elevation);

            } else {
                opMode.telemetry.addLine(String.format("Webcam Tag ID %d (Unknown)", detection.id));
            }
        }
    }

    double getBearingToAprilTag(int detectionID){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean foundit = false;
        double bearing = 0.0;
        for (AprilTagDetection detection : detections){
            if (detection.metadata != null) {
                if(detection.id == detectionID){
                    bearing = detection.ftcPose.bearing;
                    foundit = true;
                    break;
                }
            }
        }
        return bearing;
    }
    boolean getAprilTagVisible(int detectionID){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean foundit = false;
        for (AprilTagDetection detection : detections){
            if (detection.metadata != null){
                foundit = (detection.id == detectionID);
                break;
            }
        }
        return foundit;
    }
    double getDistanceToAprilTag(int detectionID){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean foundit = false;
        double distance = 0.0;
        for (AprilTagDetection detection : detections){
            if (detection.metadata != null) {
                if (detection.id == detectionID) {
                    distance = detection.ftcPose.range;
                    break;
                }
            }
        }
        return distance;
    }

    public void SetAprilTagTargetId(int value) {
        currentAprilTargetId = value;
    }
    public int GetAprilTagTargetId() { return currentAprilTargetId; }
    public boolean GetUseOnlyWebcamForDistance() { return useOnlyWebcamForDistance; }
    public void SetUseOnlyWebcamForDistance(boolean value) { useOnlyWebcamForDistance = value; }

    // *********************************************************
    // ****      LED Lights Controls and Utility functions
    // *********************************************************

    public void SetIndicatorLights() {
        // set light1 to red or blue based on alliance
        if (allianceColorString == "RED") {
            light1.setPosition(INDICATOR_RED);
        }
        else if (allianceColorString == "BLUE") {
            light1.setPosition(INDICATOR_BLUE);
        }
        else {
            light1.setPosition(INDICATOR_VIOLET);
        }

        // set light2 based on if ball in firing position
        if (lifterServo.getPosition() == LIFTER_MID_POSITION){
            light2.setPosition(INDICATOR_GREEN);
        }
        else{
            light2.setPosition(0);
        }

        // set light3 based on if shooter motor within tolerances and if turret is aligned
        double currentMotorRPM = CalculateMotorRPM(shooterMotorLeft.getVelocity(), YELLOW_JACKET_1_1_TICKS);
        boolean shooterReady =  ((currentMotorRPM >= shooterTargetSpeed - 10) && (currentMotorRPM <= shooterTargetSpeed + 200));
        boolean turretReady = ((turretMotor.getCurrentPosition() > (turretMotor.getTargetPosition() - 5)) && (turretMotor.getCurrentPosition() < (turretMotor.getTargetPosition() + 5)));  // 5 ticks is a little bit more than 2 degrees
        if (shooterReady && turretReady) {
            light3.setPosition(INDICATOR_GREEN);
        }
        else if (shooterReady) {
            light3.setPosition(INDICATOR_ORANGE);
        }
        else if (turretReady) {
            light3.setPosition(INDICATOR_YELLOW);
        }
        else {
            light3.setPosition(INDICATOR_BLACK);
        }

    }
    public void InitBlinkin(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class,"RevBLinkinLedDriver");
        /*
        if(allianceColorIsBlue)
        {
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }
        else
        {
            Blinken_pattern  = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }
        */

        Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blinkinLedDriver.setPattern(Blinken_pattern);
    }

    // *************************************************************************
    //      Shooter (spinning wheels) functions
    // *************************************************************************

    public double CalculateInitialVelocity(double distanceToGoalInMeters){
        double speedInmetersPerSecond = 0.0;
        double topPart = 0.0;
        double bottomPart = 0.0;
        topPart = 9.81 * distanceToGoalInMeters;
        bottomPart = 2 * Math.pow(Math.cos(59),2) * (distanceToGoalInMeters * Math.tan(59) + 0.98425 - 0.4064 + 0.127);
        speedInmetersPerSecond = Math.sqrt(topPart/bottomPart);
        return speedInmetersPerSecond;
        /*
        9.81 is gravitational force
        22.5 is angle of launch
        0.98425 - 0.4064 + 0.127 is height of goal - height of launch + minimal launch distance
        https://www.desmos.com/calculator/n9syawsgk2
         */
    }
    public double CalculateInitialWheelRotationalVelocity(double initialVelocityInMetersPerSecond){
        double initalWheelRotationalVelocityInRadiansPerSecond = 0.0;
        double insideParenthesis = 0.0;
        double outsideParenthesis = 0.0;
        insideParenthesis = 1 + ((1 + 0.4) / 4 * 0.5) * 0.0748 / 0.285;
        outsideParenthesis = 2 * initialVelocityInMetersPerSecond / 0.0508;
        initalWheelRotationalVelocityInRadiansPerSecond = insideParenthesis * outsideParenthesis;
        /*
        The 0.4 is a estimation for the shape factor of the projectile.
        The 0.5 is a estimation for the shape factor of the wheel launcher.
        The 0.0748 and 0.0185 are a good estimate for the mass of the projectile and the mass of the wheel in that order.
        The last value in outsideParenthesis is the radius of the wheel.
         */
        return initalWheelRotationalVelocityInRadiansPerSecond;
    }
    public void SetAutoShooterMode(boolean value){
        autoShooterMode = value;
    }
    public boolean GetAutoShooterMode() { return autoShooterMode; }
    public void SetShooterRPMFromWebCam(int currentTargetId){

        if (getAprilTagVisible(currentTargetId)){
            SetShooterMotorToSpecificRPM(CalculateOptimumShooterRPM(getDistanceToAprilTag(currentTargetId)));
            currentAprilTargetId = currentTargetId;
        }
    }
    public double CalculateOptimumShooterRPM(double distanceInInches){
        double optimumShooterRPM = 0;
        double testDistance = 24;
        while (Math.min(distanceInInches,testDistance) != distanceInInches){
            if (testDistance != 84) {
                testDistance += 6;
            }
            else {
                testDistance = 120;
                break;
            }
        }
        if      (testDistance == 24){  optimumShooterRPM = 2100; }
        else if (testDistance == 30){  optimumShooterRPM = 2200; }
        else if (testDistance == 36){  optimumShooterRPM = 2300; }
        else if (testDistance == 42){  optimumShooterRPM = 2350; }
        else if (testDistance == 48){  optimumShooterRPM = 2350; }
        else if (testDistance == 54){  optimumShooterRPM = 2400; }
        else if (testDistance == 60){  optimumShooterRPM = 2650; }
        else if (testDistance == 66){  optimumShooterRPM = 2750; }
        else if (testDistance == 72){  optimumShooterRPM = 2800; }
        else if (testDistance == 78){  optimumShooterRPM = 2950; }
        else if (testDistance == 120){ optimumShooterRPM = 3500; }
        else {
            optimumShooterRPM = 2500;
        }
        return optimumShooterRPM;
    }
    public void SetShooterSpeed(double percent){
        shooterTargetSpeed = Math.min(percent,MAX_SHOOTER_SPEED);
        shooterTargetSpeed = Math.max(percent,MIN_SHOOTER_SPEED);
        shooterMotorRight.setPower(shooterTargetSpeed);
        shooterMotorLeft.setPower(shooterTargetSpeed);
    }

    double CalculateMotorRPM (double motorVelocity, double motorCountsPerRevolution)
    {
        return (motorVelocity / motorCountsPerRevolution) * 60.0;
    }

    public void SetShooterMotorToSpecificRPM(double desiredRPM){
        shooterTargetRPM = desiredRPM;
        shooterTargetRPM = Math.min(shooterTargetRPM,MAX_SHOOTER_RPM);
        shooterTargetRPM = Math.max(shooterTargetRPM, MIN_SHOOTER_RPM);
        shooterMotorLeft.setVelocity(shooterTargetRPM * YELLOW_JACKET_1_1_TICKS / 60);
        shooterMotorRight.setVelocity(shooterTargetRPM * YELLOW_JACKET_1_1_TICKS / 60);
    }
    public double CalculateDistanceToTarget(double currentXInInches, double currentYInInches, double targetXInInches, double targetYInInches) {
        double deltaX = targetXInInches - currentXInInches;
        double deltaY = targetYInInches - currentYInInches;

        return Math.hypot(deltaX, deltaY);
    }
    public void SetShooterRPMFromPinpoint(){
        //setPinpointPositionFromWebcam();
        //pinpoint.update();
        //TODO Swich this calculation to using the new absolute positions or maybe roadrunner positions if those are better
        double distanceToTarget = CalculateDistanceToTarget(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),targetX,targetY);

        SetShooterMotorToSpecificRPM(CalculateOptimumShooterRPM(distanceToTarget));
    }
    public void SetShooterRPMFromRoadrunner(){
        double distanceToTarget = CalculateDistanceToTarget(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y,targetX,targetY);

        SetShooterMotorToSpecificRPM(CalculateOptimumShooterRPM(distanceToTarget));
    }


    // *************************************************************************
    //      Intake Functions
    // *************************************************************************

    public boolean GetAutoIntakeMode() { return autoIntakeMode; }
    public void SetAutoIntakeMode(boolean value) { autoIntakeMode = value; }
    public void SetIntakeMotor(boolean on,boolean intake){
        if (on && intake) {
            intakeMotor.setPower(INTAKE_POWER);
        }
        else if (!on){
            intakeMotor.setPower(0);
        }
        else {
            intakeMotor.setPower(-INTAKE_POWER);
        }
    }
    public void RunAutoIntake()
    {
        if (beamBreak1.isPressed()){
            intakeMotor.setPower(INTAKE_POWER);
    }
        else {
            intakeMotor.setPower(0);
        }
        //TODO -- hook up to sensors to detect balls
        /* Integrate logic to use the ball distance detection sensors to turn intake on/off automatically
         if (inside sensor says empty && outside sensor ball present) then turn intake on
         else if (inside sensor says ball present) then turn intake off

         Example code for sensor is at:
            FtcRobotController -> external.samples -> SensorDigitalTouch */
    }

    // *************************************************************************
    //      Rotating Turret Functions
    // *************************************************************************

    public void SetTurretRotationAngle(double degrees){
        // normalize the angle to be -180 to +180
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        turretTargetAngle = Math.min(degrees,MAX_TURRET_ANGLE);
        turretTargetAngle = Math.max(degrees,MIN_TURRET_ANGLE);
        turretTargetTicks = Math.round((float)turretTargetAngle * (float)TURRET_TICKS_IN_DEGREES);
        turretMotor.setTargetPosition(turretTargetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);
    }

    double getCurrentTurretAngle (){
        return turretMotor.getCurrentPosition()/TURRET_TICKS_IN_DEGREES;
    }

    public boolean GetTurretAutoMode() { return autoTurretMode; }
    public void SetTurretAutoMode(boolean value) { autoTurretMode = value; }
    void adjustTurretToTargetAprilTag(){
        double bearingToAprilTag = 0.0;
        double currentTurretAngle = getCurrentTurretAngle();
        // normalize the turret angle to be -180 to +180
        while (currentTurretAngle > 180) currentTurretAngle -= 360;
        while (currentTurretAngle < -180) currentTurretAngle += 360;
        // if can see the april tag, just use the bearing to that
        if (getAprilTagVisible(currentAprilTargetId)){
            bearingToAprilTag = getBearingToAprilTag(currentAprilTargetId);
            SetTurretRotationAngle(bearingToAprilTag + currentTurretAngle);
        }
        // else can't see the april tag, try using the roadrunner position and robot heading to calculate the turret angle
        else if (useRoadrunnerForTurretAnglesEnabled) {  //TODO need to enable this flag and test if really works
            // calculate the bearing from the front of the robot to the april tag
            bearingToAprilTag = calculateBearingToPointInDegrees(drive.localizer.getPose().position.x , drive.localizer.getPose().position.y, targetX, targetY, Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            // convert that bearing into a turret angle (turret zero position points the opposite direction of the robot)
            bearingToAprilTag += 180; 
            // normalize the bearing to be -180 to +180
            while (bearingToAprilTag > 180) bearingToAprilTag -= 360;
            while (bearingToAprilTag < -180) bearingToAprilTag += 360;
            // Command the turret to turn to that angle
            SetTurretRotationAngle(bearingToAprilTag);
        }
    }
    public static double calculateBearingToPointInDegrees(double robotXInInches, double robotYInInches, double targetXInInches, double targetYInInches, double robotHeadingDegrees) {
        double deltaX = targetXInInches - robotXInInches;
        double deltaY = targetYInInches - robotYInInches;
        double angleToTargetDeg = Math.toDegrees(Math.atan2(deltaY, deltaX));
        //double turretRelativeAngleDeg = angleToTargetDeg - robotHeadingDegrees;
        //while (turretRelativeAngleDeg > 180) turretRelativeAngleDeg -= 360;
        //while (turretRelativeAngleDeg < -180) turretRelativeAngleDeg += 360;
        //return turretHeading - robotHeadingDegrees - turretRelativeAngleDeg;
        return angleToTargetDeg - robotHeadingDegrees;  //TODO not sure if this should be + or -
    }

    public boolean GetUseRoadrunnerForTurretAnglesEnabled() { return useRoadrunnerForTurretAnglesEnabled; }
    public void SetUseRoadrunnerForTurretAnglesEnabled(boolean value) { useRoadrunnerForTurretAnglesEnabled = value; }

    // *************************************************************************
    //      Lifter Arm Functions
    // *************************************************************************

    public void SetLifterPosition(float position){
        lifterTargetPosition = Math.min(position,LIFTER_UP_POSITION);
        lifterTargetPosition = Math.max(position,LIFTER_DOWN_POSITION);
        lifterServo.setPosition(lifterTargetPosition);
    }

    public boolean GetAutoLifterMode() { return autoLifterMode; }
    public void SetAutoLifterMode(boolean value) { autoLifterMode = value; }
    public void RunAutoLifter() {
        if (autoLifterMode) {
            if (((lifterServo.getPosition() <= LIFTER_DOWN_POSITION))) {
                if (((ColorSensorRight.getDistance(DistanceUnit.INCH) > 0))
                        && (ColorSensorRight.getDistance(DistanceUnit.INCH) < 1.5)) {
                    opMode.telemetry.addData("Distance (inch)", ColorSensorRight.getDistance(DistanceUnit.INCH));
                    lifterServo.setPosition(LIFTER_MID_POSITION);
                    light1.setPosition(INDICATOR_GREEN);
                } else if (((ColorSensorLeft.getDistance(DistanceUnit.INCH) > 0))
                        && (ColorSensorLeft.getDistance(DistanceUnit.INCH) < 1.5)) {
                    opMode.telemetry.addData("Distance (inch)", ColorSensorRight.getDistance(DistanceUnit.INCH));
                    lifterServo.setPosition(LIFTER_MID_POSITION);
                    light1.setPosition(INDICATOR_GREEN);
                }
            }
        }
    }
    public void SetLifterUp(){
        SetLifterPosition(LIFTER_UP_POSITION);
        //opMode.sleep(10);
        //SetLifterPosition(LIFTER_DOWN_POSITION);
    }
    public void SetLifterDown(){

        SetLifterPosition(LIFTER_DOWN_POSITION);
    }

    // *************************************************************************
    //      Drive Control Functions
    // *************************************************************************

    public void driveControlsRobotCentric() {
        double y = -opMode.gamepad1.left_stick_y;
        double x = opMode.gamepad1.left_stick_x * 1.1;
        double rx = opMode.gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    public double GetIMU_HeadingInDegrees()
    {
        return AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + imuOffsetInDegrees);

    }
    public void driveControlsFieldCentric() {
        double y = -opMode.gamepad1.left_stick_y;
        double x = opMode.gamepad1.left_stick_x;
        double rx = opMode.gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
    public void RunDriveControls() {
        if (IsFieldCentric) {
            driveControlsFieldCentric();
        }
        else {
            driveControlsRobotCentric();
        }
    }
    public void SetFieldCentricMode(boolean fieldCentricEnabled) {
        IsFieldCentric = fieldCentricEnabled;
    }
    public void SpecialSleep(long milliseconds) {
        for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(milliseconds); stop > System.nanoTime(); ) {
            if (!opMode.opModeIsActive() || opMode.isStopRequested()) return;
            if (IsDriverControl) {
                if (IsFieldCentric) driveControlsFieldCentric();
                if (!IsFieldCentric) driveControlsRobotCentric();
            }
            //if(pidfSlidesEnabled || pidfEnabled){
            //    opMode.sleep(5);
            //}
        }
    }

}
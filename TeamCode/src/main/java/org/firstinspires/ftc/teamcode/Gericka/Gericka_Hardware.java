package org.firstinspires.ftc.teamcode.Gericka;

import static org.firstinspires.ftc.teamcode.Gericka.Gericka_MecanumDrive.PARAMS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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

    //pidf rotator variables
    public static boolean pidfEnabled = false;
    public static double p = 0.004, i = 0, d = 0, f = 0.007; //0.0001 > p was .005
    PIDController Right_controller = new PIDController(p, i, d);
    PIDController Left_controller = new PIDController(p, i, d);

    //pidf slide extension variables
    public static boolean pidfSlidesEnabled = false;
    public static double Kp = 0.0035, Ki = 0, Kd = 0.00021, Kf = 0;
    public static double Ktolerance = 5.0;
    PIDController RightSlide_controller = new PIDController(Kp, Ki, Kd);
    PIDController LeftSlide_controller = new PIDController(Kp, Ki, Kd);

    public double finalX;
    public double finalY;

    //Webcam
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    final double INTAKE_POWER = 0.5;

    public final float LIFTER_UP_POSITION = 0.85f;
    public final float LIFTER_MID_POSITION = 0.5f;
    public final float LIFTER_DOWN_POSITION = 0.05f;
    final float MAX_SHOOTER_SPEED = 1.0f;
    final float MIN_SHOOTER_SPEED = 0.0f;
    double shooterTargetRPM = 0.0;
    final float MAX_SHOOTER_RPM = 4500;
    final float MIN_SHOOTER_RPM = 0;
    final float MAX_TURRET_ANGLE = 179;
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
    public Servo allianceIndicatorLight = null;
    final double INDICATOR_BLACK = 0;
    final double INDICATOR_RED = 0.279;
    final double INDICATOR_BLUE = 0.611;
    final double INDICATOR_WHITE = 1;
    final double INDICATOR_GREEN = 0.500;
    double indicatorLightValue = 0;
    public final double FEET_TO_METER = 0.3048;
    public final double METER_TO_FEET = 3.28084;
    public final double RADIANS_PER_SECOND_TO_RPM = 9.54929658551; // 60 / (2 * Math.PI)
    public int currentAprilTargetId = 20;
    boolean autoShooterMode = false;
    GoBildaPinpointDriver pinpoint;

    RevBlinkinLedDriver.BlinkinPattern Blinken_pattern;
    RevBlinkinLedDriver blinkinLedDriver;

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
        allianceIndicatorLight = hardwareMap.get(Servo.class, "IndicatorLight");
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
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setPower(0.0);
        shooterMotorLeft.setPower(0.0);
        shooterMotorRight.setPower(0.0);
        turretMotor.setPower(0.0);



        // ****************** SERVOS ******************************************
        lifterServo = hardwareMap.get(Servo.class, "lifterServo");



        // ********** Color Sensors ********************

        //leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");
        //rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorRight");
        //leftColorSensor.enableLed(false);
        //rightColorSensor.enableLed(false);
        if (Objects.equals(allianceColor, "RED")) {
            allianceIndicatorLight.setPosition(INDICATOR_RED);
        } else {
            allianceIndicatorLight.setPosition(INDICATOR_BLUE);
        }

         //limelightbox = hardwareMap.get(Limelight3A.class, "limelight");
        //InitBlinkin(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        imu.initialize(parameters);
        imu.resetYaw();

    }

    public void InitRoadRunner(HardwareMap hardwareMap)
    {
        drive = new Gericka_MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

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
        //TODO need to verify X,Y directions, going forward should make X count up, going Left should make Y count up
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
        opMode.telemetry.addData("pinpoint Position(inches): ", "x: %.1f, y: %.1f", pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
        opMode.telemetry.addData("pinpoint Heading(deg): ", pinpoint.getHeading(AngleUnit.DEGREES));

    }

    public void SetPinpointPosition(double xPositionInches, double yPositionInches, double headingDegrees) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xPositionInches, yPositionInches, AngleUnit.DEGREES, headingDegrees));

        pinpoint.update();
    }

   public void WebcamInit (HardwareMap hardwareMap){
       aprilTag = new AprilTagProcessor.Builder().build();

       VisionPortal.Builder builder = new VisionPortal.Builder();
       builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
       builder.addProcessor(aprilTag);

       visionPortal = builder.build();
        // Create the TensorFlow processor the easy way.
        // = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.

        //     visionPortal = VisionPortal.easyCreateWithDefaults(
        //             hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
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


    public void SetIndicatorLight(double colorValue) {
        allianceIndicatorLight.setPosition(colorValue);
        indicatorLightValue = colorValue;
    }
    // *********************************************************
    // ****      BLINKIN LED Lights Controls                ****
    // *********************************************************

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
    public double initialVelocityCalculator(double distanceToGoalInMeters){
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
    public double initialWheelRotationalVelocityCalculator(double initialVelocityInMetersPerSecond){
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
    public void ShooterRPMFromWebCam(int currentTargetId){

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
        if (testDistance == 24){
            optimumShooterRPM = 2100;
        }
        else if (testDistance == 30){
            optimumShooterRPM = 2200;
        }
        else if (testDistance == 36){
            optimumShooterRPM = 2300;
        }
        else if (testDistance == 42){
            optimumShooterRPM = 2350;
        }
        else if (testDistance == 48){
            optimumShooterRPM = 2350;
        }
        else if (testDistance == 54){
            optimumShooterRPM = 2400;
        }
        else if (testDistance == 60){
            optimumShooterRPM = 2650;
        }
        else if (testDistance == 66){
            optimumShooterRPM = 2750;
        }
        else if (testDistance == 72){
            optimumShooterRPM = 2800;
        }
        else if (testDistance == 78){
            optimumShooterRPM = 2950;
        }
        else if (testDistance == 120){
            optimumShooterRPM = 3500;
        }
        else {
            optimumShooterRPM = 2500;
        }
        return optimumShooterRPM;
    }
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
    public void SetTurretRotationAngle(double degrees){
        turretTargetAngle = Math.min(degrees,MAX_TURRET_ANGLE);
        turretTargetAngle = Math.max(degrees,MIN_TURRET_ANGLE);
        turretTargetTicks = Math.round((float)turretTargetAngle * (float)TURRET_TICKS_IN_DEGREES);
        turretMotor.setTargetPosition(turretTargetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);
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
            }
        }
        return foundit;
    }
    double getCurrentTurretAngle (){
        double turretAngle = turretMotor.getCurrentPosition()/TURRET_TICKS_IN_DEGREES;

        return turretAngle;
    }

    void adjustTurretToTarget(int detectionID){
        if (getAprilTagVisible(detectionID)){
            double x = getBearingToAprilTag(detectionID);
            double y = getCurrentTurretAngle();
            SetTurretRotationAngle(x+y);
        }
    }
    double getDistanceToAprilTag(int detectionID){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean foundit = false;
        double distance = 0.0;
        for (AprilTagDetection detection : detections){
            if (detection.metadata != null) {
                distance = detection.ftcPose.range;

                }
            }
        return distance;
    }

    public void SetLifterPosition(float position){
        lifterTargetPosition = Math.min(position,LIFTER_UP_POSITION);
        lifterTargetPosition = Math.max(position,LIFTER_DOWN_POSITION);
        lifterServo.setPosition(lifterTargetPosition);
    }
    public void SetLifterUp(){
        SetLifterPosition(LIFTER_UP_POSITION);
        //opMode.sleep(10);
        //SetLifterPosition(LIFTER_DOWN_POSITION);
    }
    public void SetLifterDown(){
        SetLifterPosition(LIFTER_DOWN_POSITION);
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

    public void ShowTelemetry(){

        opMode.telemetry.addData("Shooter ", "L-RPM: %.1f, R-RPM: %.1f", CalculateMotorRPM(shooterMotorLeft.getVelocity(), YELLOW_JACKET_1_1_TICKS), CalculateMotorRPM(shooterMotorRight.getVelocity(), YELLOW_JACKET_1_1_TICKS));
        opMode.telemetry.addData("        ", "L-Vel: %.1f, R-Vel: %.1f" , shooterMotorLeft.getVelocity(), shooterMotorRight.getVelocity());
        opMode.telemetry.addData("        ", "L-Pow: %.1f, R-Pow: %.1f" , shooterMotorLeft.getPower(), shooterMotorRight.getPower());
        opMode.telemetry.addData("        ", "L-Amp: %.1f, R-Amp: %.1f" , shooterMotorLeft.getCurrent(CurrentUnit.AMPS), shooterMotorRight.getCurrent(CurrentUnit.AMPS));
        opMode.telemetry.addData("Shooter Target Speed: ", shooterTargetSpeed);
        opMode.telemetry.addData("Shooter Target RPM", shooterTargetRPM);
        opMode.telemetry.addData("April Tag Target ID", currentAprilTargetId);
        opMode.telemetry.addData("Auto Shooter Mode",autoShooterMode);

        opMode.telemetry.addData("Turret ", "ticks: %d, tgt-Angle: %.1f", turretMotor.getCurrentPosition(),turretTargetAngle);
        opMode.telemetry.addData("       ", "Pow: %.1f, Amp: %.1f", turretMotor.getPower(),turretMotor.getCurrent(CurrentUnit.AMPS));

        opMode.telemetry.addData("Intake ", "Pow: %.1f, Amp: %.1f", intakeMotor.getPower(), intakeMotor.getCurrent(CurrentUnit.AMPS));
        opMode.telemetry.addData("       ", "Vel: %.1f", intakeMotor.getVelocity());

        opMode.telemetry.addData("Lifter Position: ", lifterServo.getPosition());

        opMode.telemetry.addData("imu Heading: ", GetIMU_HeadingInDegrees());
        //opMode.telemetry.addData("imu roll: ", (imu.getRobotYawPitchRollAngles().getRoll()));
        //opMode.telemetry.addData("imu pitch: ", (imu.getRobotYawPitchRollAngles().getPitch()));
        //opMode.telemetry.addData("imu yaw: ", (imu.getRobotYawPitchRollAngles().getYaw()));

        ShowPinpointTelemetry();

        if (allianceIndicatorLight.getPosition() == INDICATOR_RED)
        {
            opMode.telemetry.addData("Alliance: ", "RED");
        }
        else if (allianceIndicatorLight.getPosition() == INDICATOR_BLUE)
        {
            opMode.telemetry.addData("Alliance: ", "BLUE");
        }
        else {
            opMode.telemetry.addData("ERROR: Alliance Value:", allianceIndicatorLight.getPosition());
        }

        telemetryAprilTag();

        //opMode.telemetry.addData("lastStatusMsg: ", lastStatusMsg);
        //opMode.telemetry.addData("lastErrorMsg: ", lastErrorMsg);
        //opMode.telemetry.addData("PIDF Enabled:", pidfEnabled);

        //opMode.telemetry.addData("Auto Last Time Left: ", autoTimeLeft);

        opMode.telemetry.update();
    }

    private String lastStatusMsg = "None";
    private String lastErrorMsg = "None";

    public void SetLastStatusMsg(String newMsg)
    {
        lastStatusMsg = newMsg;
    }
    public void SetLastErrorMsg(String newMsg)
    {
        lastErrorMsg = newMsg;
    }

    /*
    public void moveRobot(double x, double y, double yaw) {
        // opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
        //  opMode.telemetry.update();
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower *.5);
        rightFront.setPower(rightFrontPower *.5);
        leftRear.setPower(leftBackPower *.5);
        rightRear.setPower(rightBackPower *.5);

        //  opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
        //  opMode.telemetry.update();
    }

     */

    public void EndgameBuzzer(){
        if(opMode.getRuntime() < 84.5 && opMode.getRuntime() > 84.0){
            opMode.gamepad1.rumble(1000);
            opMode.gamepad2.rumble(1000);
        }
    }
    //opMode.getRuntime() < 109.5 && opMode.getRuntime() > 109.0       10 SECONDS


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

    /*
    public void driveControlsRobotCentricKID() {
        double y = -opMode.gamepad2.left_stick_y;
        double x = opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower*.25);
        leftRear.setPower(backLeftPower*.25);
        rightFront.setPower(frontRightPower*.25);
        rightRear.setPower(backRightPower*.25);
    }

     */

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
            if(pidfSlidesEnabled || pidfEnabled){
                opMode.sleep(5);
            }
        }
    }

    public boolean isRobotLevel() {
        boolean returnValue = false;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        imuPosition = (orientation.getRoll());
        if(imuPosition < 2 || imuPosition > -2)
        {
            returnValue = true;
        }
        return returnValue;

    }

}
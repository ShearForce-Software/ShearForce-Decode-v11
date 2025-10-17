package org.firstinspires.ftc.teamcode.Gericka;

import static org.firstinspires.ftc.teamcode.Geronimo.MecanumDrive_Geronimo.PARAMS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Geronimo.MecanumDrive_Geronimo;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
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

    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;

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
    public void Init (HardwareMap hardwareMap) {
        // ************* Drive MOTORS ****************
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront_leftOdometry");
        leftRear= hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear_rightOdometry");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront_centerOdometry");

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);


        // ********** Color Sensors ********************

        //leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");
        //rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorRight");
        //leftColorSensor.enableLed(false);
        //rightColorSensor.enableLed(false);


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



    /*
    public void AlignOnFloorSampleWithPercent() {

        // 1.  Classic pixel-angle based offset
        double[] offsetInches = GetStrafeOffsetInInches(limelight_targetImageName);

        // 2.  How much of the block is visible right now?
        double pct = GivePercentOfTarget();              // 0 – 100
        pct = Math.max(0.0, Math.min(pct, MAX_SEEN_AREA_PCT)); // clamp just in case

        // 3.  Compute the bias term (larger when pct is small, 0 when pct == MAX)
        double visibilityFactor = (MAX_SEEN_AREA_PCT - pct) / MAX_SEEN_AREA_PCT; // 0 … 1
        //      ^^^ 1 when nothing seen
        // Apply the same scaling to X and Y, keeping their directions
        double biasX = Math.signum(offsetInches[0]) * visibilityFactor * AREA_OFFSET_SCALE_INCH;
        double biasY = Math.signum(offsetInches[1]) * visibilityFactor * AREA_OFFSET_SCALE_INCH;

        // 4.  Final commanded offsets
        double finalX = offsetInches[0] + biasX;   // remember: X  = left/right
        double finalY = offsetInches[1] + biasY;   //            Y  = forward/back

        // 5.  If no target at all (== our -1 flag propagated), skip strafe
        if (Math.abs(offsetInches[0]) < 0.001 && Math.abs(offsetInches[1]) < 0.001) {
            opMode.telemetry.addLine("NO TARGET");
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(Blinken_pattern);
            return;
        }

        // 6.  Same LED logic as before
        switch (limelight_targetImageName) {
            case "red":
                Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;   break;
            case "yellow":
                Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW; break;
            case "blue":
                Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;   break;
            default:
                Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;  break;
        }
        blinkinLedDriver.setPattern(Blinken_pattern);

        // 7.  Build the Road Runner action
        drive.updatePoseEstimate();
        Pose2d  currentPose  = drive.pose;
        Vector2d targetVector = new Vector2d(-finalY, finalX); // note X/Y swap!

        Action strafeAction = drive.actionBuilder(currentPose)
                .strafeToConstantHeading(targetVector)
                .build();

        Actions.runBlocking(strafeAction);
    }
*/



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


/*
    public void NavToTag(){
        desiredTag  = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == DESIRED_TAG_ID) ){
                desiredTag = detection;
                rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                yawError        = desiredTag.ftcPose.yaw;
                break;  // don't look any further.
            } else {
                opMode.telemetry.addData("Unknown Target - ", "No Tag Detected");
                // set some range and yaw error
            }
        }
    }
*/

/*
    public void DriveToTag() {
        double drive = 0.0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0.0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0.0;        // Desired turning power/speed (-1 to +1)

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        NavToTag();
        double headingError = desiredTag.ftcPose.bearing;

        double timeout = opMode.getRuntime() + 5;

        while ((rangeError > DESIRED_DISTANCE) &&
                ((headingError > 2.0) || (headingError < -2.0)) &&
                ((yawError > 2.0) || (yawError < -2.0)) && (opMode.getRuntime() < timeout)) {
            // Determine heading, range and Yaw (tag image rotation) errors
            NavToTag();
            headingError = desiredTag.ftcPose.bearing;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            // Speed has been reduced for the AirShow on June 9th
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            opMode.telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            opMode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            opMode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            opMode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            opMode.telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            opMode.telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            opMode.sleep(10);
        }
    }

 */

    // *********************************************************
    // ****      Color Sensor Methods                       ****
    // *********************************************************
/*
    protected void InitColorRevV3Sensor() {
        float gain = 51;
        final float[] hsvValues = new float[3];
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;
        if (leftColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) leftColorSensor).enableLight(true);
        }


    }
 */

    // colorFound loop
    /*
    public boolean ColorRevV3SensorChecker(colorEnum targetColor) {
        boolean colorFound = false;
        double breakLoop = 10 + opMode.getRuntime();
        while (!colorFound && opMode.getRuntime() <= breakLoop) {
            ColorRevV3Sensor();
            if (colorDetected == targetColor) {
                colorFound = true;
            }
            opMode.sleep(100);
        }
        return colorFound;
    }
    */

    /*
    // returns colorEnum color detected
    float gain = 51;
    float[] hsvValues = {0,0,0};
    public colorEnum ColorRevV3Sensor() {
        leftColorSensor.setGain(gain);
        NormalizedRGBA colors = leftColorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Red HSV Color ranges
        // changed
        double hMinRed = 18.800;
        double hMaxRed = 128.000;
        double sMinRed = 0.3;
        double sMaxRed = 0.7235;
        double vMinRed = 0.263;
        double vMaxRed = 0.722;

        // Yellow HSV Color Values
        double hMinYellow = 59.000;
        double hMaxYellow = 113.043;
        double sMinYellow = 0.501;
        double sMaxYellow = 0.772;
        double vMinYellow = 0.565;
        double vMaxYellow = 1.000;

        // Blue HSV Color Values
        double hMinBlue = 187.152;
        double hMaxBlue = 219.568;
        double sMinBlue = 0.741;
        double sMaxBlue = 0.832;
        double vMinBlue = 0.514;
        double vMaxBlue = 1.000;

        // determine if color is blue, red or yellow and show telemetry
        if (hsvValues[0] >= hMinBlue && hsvValues[1] >= sMinBlue)
        {
            colorDetected = colorEnum.blue;
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }


        else if (hsvValues[1] <= sMaxYellow && hsvValues[1] >= sMinYellow && hsvValues[0] <= hMaxYellow && hsvValues[0] >= hMinYellow)
        {
            colorDetected = colorEnum.yellow;
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }

        else if (hsvValues[1] <= sMaxRed && hsvValues[1] >= sMinRed && hsvValues[0] <= hMaxRed && hsvValues[0] >= hMinRed)
        {
            colorDetected = colorEnum.red;
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }

        else
        {
            colorDetected = colorEnum.noColor;
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }

        return colorDetected;

    }

     */

    /*
    public void showColorSensorTelemetry(){
        //int leftColor = leftColorSensor.getNormalizedColors().toColor();
        //opMode.telemetry.addData("leftColorNorm: ", leftColor);
        //opMode.telemetry.addData("leftColor: ", "red: %d, green: %d, blue: %d", redLeft, greenLeft, blueLeft);
        //opMode.telemetry.addData("rightColor: ", "red: %d, green: %d, blue: %d", redRight, greenRight, blueRight);
        //opMode.telemetry.addData("rightColor: ", rightColor);
        //opMode.telemetry.addData("leftColorNorm(red): ", leftColorSensor.getNormalizedColors().red);
        //opMode.telemetry.addData("leftColorNorm(green): ", leftColorSensor.getNormalizedColors().green);
        //opMode.telemetry.addData("leftColorNorm(blue): ", leftColorSensor.getNormalizedColors().blue);

        int red = leftColorSensor.red();
        int green = leftColorSensor.green();
        int blue = leftColorSensor.blue();
        // Check for White Pixel
        if(red < 4000 && red > 1000 && green < 6000 && green > 3000 && blue < 7000 && blue > 3000) {
            opMode.telemetry.addData("Left: ", "is white");
        }
        // Check for yellow pixel
        else if(red < 2500 && red > 1000 && green < 3500 && green > 1500 && blue < 1000 && blue >0 )
        {
            opMode.telemetry.addData("Left: ", "is yellow");
        }
        // Check for green pixel
        else if(red < 1000 && red > 0 && green < 6000 && green > 1500 && blue < 1000 && blue >0 )
        {
            opMode.telemetry.addData("Left: ", "is green");
        }
        // Check for purple pixel
        else if(red < 3500 && red > 1000 && green < 4000 && green > 2000 && blue < 7000 && blue > 3500 )
        {
            opMode.telemetry.addData("Left: ", "is purple");
        }
        else {
            opMode.telemetry.addData("Left: ", "unknown");
        }


    }
    */

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

    //public void setBlinken_to5Volt()
    //{
    //    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1625));
    //}


    public void ShowTelemetry(){

        opMode.telemetry.addData("Auto Last Time Left: ", autoTimeLeft);
        opMode.telemetry.addData("imu Heading: ", GetIMU_HeadingInDegrees());
        ;

        opMode.telemetry.addData("PIDF Enabled:", pidfEnabled);

        opMode.telemetry.addData("imu roll: ", (imu.getRobotYawPitchRollAngles().getRoll()));
        opMode.telemetry.addData("imu pitch: ", (imu.getRobotYawPitchRollAngles().getPitch()));
        opMode.telemetry.addData("imu yaw: ", (imu.getRobotYawPitchRollAngles().getYaw()));

        opMode.telemetry.addData("lastStatusMsg: ", lastStatusMsg);
        opMode.telemetry.addData("lastErrorMsg: ", lastErrorMsg);

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
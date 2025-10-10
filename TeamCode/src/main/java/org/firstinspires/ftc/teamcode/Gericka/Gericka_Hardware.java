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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Geronimo.MecanumDrive_Geronimo;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


@Config
public class Gericka_Hardware {
    //ReentrantLock lock = new ReentrantLock();
    public static double txCorrectionSensitivity = 1.18;
    public static double tyCorrectionSensitivity = 0.78;

    IMU imu;
    public double imuOffsetInDegrees = 0.0;
    double imuPosition = 0;

    MecanumDrive_Geronimo drive;

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

    final double arm_gear_ratio = 90.0/20.0;
    //final double yellow_jacket_27_ticks = 751.8;    //9.4 ticks for each degree of arm rotation
    final double yellow_jacket_51_ticks = 1425.1;   //17.81 ticks for each degree of arm rotation
    final double ticks_in_degrees = (arm_gear_ratio/360.0) * yellow_jacket_51_ticks;
    public double rotator_arm_target_ticks = 0;
    public double rotator_arm_target_angle = 0.0;

    //pidf slide extension variables
    public static boolean pidfSlidesEnabled = false;
    public static double Kp = 0.0035, Ki = 0, Kd = 0.00021, Kf = 0;
    public static double Ktolerance = 5.0;
    PIDController RightSlide_controller = new PIDController(Kp, Ki, Kd);
    PIDController LeftSlide_controller = new PIDController(Kp, Ki, Kd);

    public double finalX;
    public double finalY;





    // Slide Arm Rotators
    DcMotor leftSlideArmRotatorMotor;
    DcMotor rightSlideArmRotatorMotor;
    TouchSensor touchSensorSlideArmRotatorRight;
    TouchSensor touchSensorSlideArmRotatorLeft;
    int slideArmRotatorTargetPosition = 0;
    double slideArmRotatorPower = 0.0;
    boolean slideArmRotatorRunningToPosition = false;
    public static final int SLIDE_ARM_ROTATOR_MIN_POS = 0;
    public static final int SLIDE_ARM_ROTATOR_MAX_POS = 1640;  //  //870
    public static final int SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES = 300; // TODO need to find the lowest rotator position we can allow the slides to go out
    public static final double SLIDE_ARM_ROTATOR_POWER = 0.75;

    //slides
    DcMotor slideLeft;
    DcMotor slideRight;
    int slidesTargetPosition = 0;
    boolean slidesRunningToPosition = false;
    public static final double SLIDES_POS_POWER = 1.0;
    public static final int SLIDE_ARM_MIN_POS = 0;
    public static final int SLIDE_ARM_MAX_VERTICAL_POS = 4382; //5918 5300 //5533
    public static final int SLIDE_ARM_MAX_HORIZONTAL_POS = 2297; //1550 //1400  //3690 //3310 //2900
    //OLD- pulley = 12.1,  12.1pi = 38mm,  motor ticks = 1425.1,  1425.1/38 = 37.5 ticks/mm
    //NEW- pulley = 15.3, 15.3pi = 48mm, motor ticks = 1425.1, 1425.1/48 = 29.7 ticks/mm
    //changing it from old to new- 29.7/37.5 = 0.792, multiply old number by 0.792 to change to new
    private double slidePower = 0.0;
    TouchSensor touchSensorSlideLeft;
    TouchSensor touchSensorSlideRight;

    Servo clawServo;
    public static final double CLAW_MAX_POS = 0.4; //0.45
    public static final double CLAW_MIN_POS = 0.0;
    double claw_position = 0.5;

    Servo swiperServo;
    public static final double SWIPER_MAX_POS = 0.8;
    public static final double SWIPER_MIN_POS = 0.25;

    Servo swiper2;
    public static final double SWIPER2_MAX_POS = 0.8;
    public static final double SWIPER2_MIN_POS = 0.25;

    Servo intakeBoxRotaterServo;
    public static final double INTAKE_STAR_BOX_ROTATOR_MAX_POS = 1.0;
    public static final double INTAKE_STAR_BOX_ROTATOR_MIN_POS = 0.0;
    public static final double INTAKE_STAR_BOX_ROTATOR_INCREMENT = 0.01;
    double intakeBoxRotatorPosition = 0.5;

    Servo smallArmHangerLeftServo;
    Servo smallArmHangerRightServo;
    public static final double SMALL_ARM_HANGER_MAX_POS = 0.95;
    public static final double SMALL_ARM_HANGER_MIN_POS = 0.0;
    double smallArmHangerLeftPosition = 0.5;
    double smallArmHangerRightPosition = 0.5;
    static final double SMALL_ARM_HANGER_INCREMENT = 0.01;

    Servo lockServo1;
    Servo lockServo2;
    double lock_position = 0;       // release position

    public Servo urchinServo;
    double urchinServo_position = 0.5;
    public static final double URCHIN_SERVO_MAX_POS = 1.0;
    public static final double URCHIN_SERVO_MIN_POS = 0.0;

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

    //LimeLight
    public static boolean limelightEnabled = true;
    private Limelight3A limelightbox;

    LLResult limelight_result;//= new LLResult();
    LLStatus limelight_status = new LLStatus();
    int limelightPipelineId = 9;

    //double captureLatency = result.getCaptureLatency();
    //double targetingLatency = result.getTargetingLatency();
    //ouble parseLatency = result.getParseLatency();

    //private static final double KpDistance = -0.1; // Proportional control constant for distance adjustment
    //private static final double KpAim = 0.1; // Proportional control constant for aiming adjustment

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
        drive = new MecanumDrive_Geronimo(hardwareMap, new Pose2d(0, 0, 0));
    }
    // *********************************************************
    // ****      LIME LIGHT Methods                         ****
    // *********************************************************

    public void SetLimelightEnabled (boolean enabled)
    {
        limelightEnabled = enabled;
    }

    /* -------------------------------------------------------------------
     *  Helper -- convert a robot–relative (dx,dy) to a field vector
     * -------------------------------------------------------------------*/



    public void InitLimelight(HardwareMap hardwareMap){
        limelightbox = hardwareMap.get(Limelight3A.class, "limelight");

        // According to the limelight AI, the default is 100 Hz (fresh results every 10ms),
        // going beyond that might "overwhelm the network"
        limelightbox.setPollRateHz(100);  // TODO experiment with changing this value to be lower, like 10

        limelightbox.pipelineSwitch(limelightPipelineId);

        // the start should be called anytime the limelight has been paused/stopped or a new pipeline loaded
        // if start has not been called then getLatestResult will return a null
        limelightbox.start();

        limelight_status = limelightbox.getStatus();
        limelight_result = limelightbox.getLatestResult();
        //limelightbox.getLatestResult().getTx();
       // limelightbox.getLatestResult().getTy();
       // angletoObject = LimelightMountingAngle + (limelightbox.getLatestResult().getTy());
        //distance_to_object = (objectHeight-LimelightMountingHeight)/(Math.tan(angletoObject));
        // Equation above was pulled from the Limelight documentation online
       // XDistance_to_object = distance_to_object*Math.cos((limelightbox.getLatestResult().getTx()));
       // YDistance_to_object = distance_to_object*Math.sin((limelightbox.getLatestResult().getTx()));
    }

    //This method basically finds the amount of tx and ty angle from crosshair to target.
    //It then returns an ArrayList giving back both tx and ty values.

    public void UpdateLimelightStatusAndResults(){
        limelight_status = limelightbox.getStatus();
        limelight_result = limelightbox.getLatestResult();
        //LLResult tempResult = limelightbox.getLatestResult();
        //if(tempResult != null && tempResult.isValid()){
        //    result = tempResult;
        //}
    }

    public void SwitchLimelightPipeline() {
        if (limelightPipelineId == 9)
        {
            limelightPipelineId = 7;
            // check pipline color
            SetLimelightToYellow();
        }
        else if (limelightPipelineId == 8)
        {
            limelightPipelineId = 9;
            SetLimelightToRed();
        }
        else if (limelightPipelineId == 7)
        {
            limelightPipelineId = 8;
            // check pipline color
            SetLimelightToBlue();

        }

        switch (limelight_targetImageName) {
            case "red":    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;    break;
            case "yellow": Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW; break;
            case "blue":   Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;   break;
            default:       Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;  break;
        }
        blinkinLedDriver.setPattern(Blinken_pattern);

        limelightbox.pipelineSwitch(limelightPipelineId);
        limelightbox.start();
    }


    /**
     * Return the tx/ty from the first ColorResult in your color-threshold pipeline,
     * or NaN/NaN if nothing is seen.
     */
    public double[] getColorTxTy() {
        // 1) Check that we have a valid latest result
        if (limelight_result != null && limelight_result.isValid()) {
            List<LLResultTypes.ColorResult> cr = limelight_result.getColorResults();
            if (cr != null && !cr.isEmpty()) {
                // take the first (and only) color-range rectangle
                return new double[]{
                        cr.get(0).getTargetXDegrees(),
                        cr.get(0).getTargetYDegrees()
                };
            }
        }
        // 2) Fallback when no color‐rectangle is detected
        return new double[]{ Double.NaN, Double.NaN };
    }

    private double[] getStrafeOffsetInInchesColor() {

        double[] raw = getColorTxTy();
        if (Double.isNaN(raw[0]) || Double.isNaN(raw[1])) {
            // target not found
            return new double[] { 0.0, 0.0 };
        }

        // Convert angles → inches (same maths as before)
        final double D = 7.2;                                // camera height in in
        double strafeX =  (D * Math.tan(Math.toRadians(raw[0])) * tyCorrectionSensitivity);
        double strafeY = (D * Math.tan(Math.toRadians(raw[1])) * txCorrectionSensitivity)-1;

        return new double[] { strafeX, strafeY };
    }

    private Vector2d robotOffsetToField(double dxRobot, double dyRobot) {
        // heading of the RR pose estimate (already fused with IMU)
        double h = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //need help?????        // radians, CCW+

        double dxField =  dxRobot * Math.cos(-h) - dyRobot * Math.sin(-h);      // do we use -h or h?????
        double dyField =  dxRobot * Math.sin(-h) + dyRobot * Math.cos(-h);



        // so basically yer RoadRunner’s X-axis is field-forward, Y-axis is field-left,
        //but our Limelight “dxRobot” is strafe-right  (-Y in RR coordinates)
        //and then basicallyyy “dyRobot” is forward (+X)    ernoe.  The rotation above keeps that
         //convention; we do the same swap we basiclaly  already had later on.w
        return new Vector2d(dxField, dyField);
    }

    public void AlignOnFloorSampleWithPercent() {

        // 1  pixel-angle offset from colour box
        double[] offsetInches = getStrafeOffsetInInchesColor();

        // 2  extra bias from vi sible area %
        double pct = GivePercentOfTarget();                       // 0-100
        pct = Math.max(0.0, Math.min(pct, MAX_SEEN_AREA_PCT)); //clamping technique so nothing stupiid happens

        double visibility = (MAX_SEEN_AREA_PCT - pct) / MAX_SEEN_AREA_PCT; // 0…1
        double biasX = Math.signum(offsetInches[0]) * visibility /* * AREA_OFFSET_SCALE_INCH*/;
        double biasY = Math.signum(offsetInches[1]) * visibility /* * AREA_OFFSET_SCALE_INCH*/;

        finalX = offsetInches[0] + biasX;
        finalY = offsetInches[1] + biasY;

        // 3  abort if target not found (both ≈0)
        if (Math.abs(offsetInches[0]) < 0.25 && Math.abs(offsetInches[1]) < 0.001) {
            opMode.telemetry.addLine("NO TARGET");
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(Blinken_pattern);
            return;
        }

        // 4  LED & strafe identical to the old routine
        switch (limelight_targetImageName) {
            case "red":    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;    break;
            case "yellow": Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW; break;
            case "blue":   Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;   break;
            default:       Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;  break;
        }
        blinkinLedDriver.setPattern(Blinken_pattern);

        drive.updatePoseEstimate();
        Pose2d currentPose = drive.localizer.getPose();
        //Vector2d targetVector = new Vector2d(drive.pose.position.x -finalY, drive.pose.position.y + finalX);  // note swap

        // --- 3  convert robot offsets → field space ---
        Vector2d fieldOffset = robotOffsetToField(finalY, finalX);   // (dxF , dyF)

// --- 4  build where we really want to be basicallyyy
        Vector2d targetVector = new Vector2d(
                drive.localizer.getPose().position.x - fieldOffset.x,  // forward/back  (RR-X)
                   drive.localizer.getPose().position.y + fieldOffset.y); // strafe        (RR-Y)

        Action strafeAction = drive.actionBuilder(currentPose)
                .strafeToConstantHeading(targetVector)
                .build();

        Actions.runBlocking(strafeAction);
    }


    public List<Double>  FindAlignAngleToTargetImage(String targetImageName) {    // outdated lynx firmware error for roadrunner
        List<Double> offset = new ArrayList<>();


        if (limelight_result != null && limelight_result.isValid()) {
            // Access detector results
            List<LLResultTypes.DetectorResult> detectorResults = limelight_result.getDetectorResults();
            for (LLResultTypes.DetectorResult dr : detectorResults) {
                String BoxType = dr.getClassName();

                // Confirm once that Target Image is Found, before attempting alignment
                if (BoxType.equals(targetImageName)) {
                    double currentOffsetX = limelight_result.getTx() ;
                    double currentOffsetY = limelight_result.getTy() ;

                    //Return both offsets as a list
                    offset.add(currentOffsetX);
                    offset.add(currentOffsetY);
                    return offset;
                }
            }
        }
        //If the target is not found, add -1.0 to both offsets
        offset.add(-1.0);
        offset.add(-1.0);
        return offset;
    }

    String limelight_targetImageName = "red";
    public void SetLimelightToRed()
    {
        limelight_targetImageName = "red";
    }
    public void SetLimelightToBlue()
    {
        limelight_targetImageName = "blue";
    }
    public void SetLimelightToYellow()
    {
        limelight_targetImageName = "yellow";
    }
    public double[] GetStrafeOffsetInInches(String targetImageName) {

        // Retrieve scaled offsets (already assumed from the recognized target).
        List<Double> scaledOffsets = FindAlignAngleToTargetImage(targetImageName);

        // Check if target was found by checking for the -1.0 flag on both tx and ty
        if (scaledOffsets.get(0) == -1.0 && scaledOffsets.get(1) == -1.0) {
            // Target not found; propagate some "error" condition, [0,0].
            return new double[] {0.0, 0.0};
        }

        // Convert the scaled offsets back to raw angles
        double rawTx = scaledOffsets.get(0);
        double rawTy = scaledOffsets.get(1);

        // Fixed distance from the target in inches
        final double D = 7.4;

        // Convert angles from degrees to radians
        double txRadians = Math.toRadians(rawTx);
        double tyRadians = Math.toRadians(rawTy);

        // Calculate the strafing offsets
        double strafeX = (D * Math.tan(txRadians) * tyCorrectionSensitivity) ; // Left/Right adjustment
        double strafeY = (D * Math.tan(tyRadians) * txCorrectionSensitivity) -2.3; // Forward/Backward adjustment

        // Return the offsets in a double array
        return new double[] {strafeX, strafeY};
    }



    public void AlignOnFloorSample()
    {
        double [] offsetInches = GetStrafeOffsetInInches(limelight_targetImageName);

        if(Math.abs(offsetInches[0])<0.001){
            opMode.telemetry.addLine("NOPE");
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }
        else{
            switch (limelight_targetImageName) {
                case "red":
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    break;
                case "yellow":
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    break;
                case "blue":
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                    break;
                default:
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    break;
            }
            blinkinLedDriver.setPattern(Blinken_pattern);

            drive.updatePoseEstimate();//Update the current roadrunner pose estimate
            //TODO -- double fixedForwardAdjustment = 1.5;

            Pose2d currentPose = drive.localizer.getPose();
            Vector2d targetVector = new Vector2d(-offsetInches[1] , offsetInches[0]);


            Action strafeAction = drive.actionBuilder(currentPose)
                    .strafeToConstantHeading(targetVector)
                    .build();

            Actions.runBlocking(strafeAction);
        }
    }

    public static final double MAX_SEEN_AREA_PCT= 17.0;
    public static double AREA_OFFSET_SCALE_INCH = 4.0;

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



    public double GivePercentOfTarget() {

        // Make sure the Limelight has been initialised
        if (limelightbox == null) {
            return 0.0;
        }

        // Ask for the latest frame’s result
        LLResult result = limelightbox.getLatestResult();

        // Sanity-check: non-null result object AND target found
        if (result != null && result.isValid()) {
            return result.getTa();     // already 0–100 %
        }

        // Fallback if no valid contour in view
        return 0.0;
    }

    public boolean limelightHasTarget() {
        LLResult result = limelightbox.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult colorResult : colorResults) {
                // Example: Check if Limelight sees a color target
                if (colorResult.getTargetXDegrees() != 0 && colorResult.getTargetYDegrees() != 0) {
                    opMode.telemetry.addData("Color Detected", "X: %.2f, Y: %.2f", colorResult.getTargetXDegrees(), colorResult.getTargetYDegrees());
                    return true;
                }
            }
        }
        return false;
    }

    /*
    public boolean limelightHasCustomTarget() {
        LLResult result = limelightbox.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            if (detectorResults != null && !detectorResults.isEmpty()) {
                for (LLResultTypes.DetectorResult detectorResult : detectorResults) {
                    // Optionally, check for a specific class ID if you have multiple objects
                    //int classID = detectorResult.getClassID();
                    double confidence = detectorResult.getConfidence();
                    //opMode.telemetry.addData("Object Detected", "Class ID: %d, Confidence: %.2f", classID, confidence);
                    // If you want to check for a specific object, replace 'YOUR_CUSTOM_OBJECT_CLASS_ID' with your object's class ID
                    // if (classID == YOUR_CUSTOM_OBJECT_CLASS_ID) {
                    return true;
                    // }
                }
            }
        }
        return false;
    }

     */

/*    public void WebcamInit (HardwareMap hardwareMap){
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        // Create the TensorFlow processor the easy way.
        // = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.

        //     visionPortal = VisionPortal.easyCreateWithDefaults(
        //             hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
    }

 */

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

    // *********************************************************
    // ****      COMBO MOVES                                ****
    // *********************************************************

    public void AutoStartPosition()
    {
        SetIntakeBoxRotatorPosition(0.805); //0.875
        SetSmallArmHangerPosition(0.1); //0 //.25
        SetSlideToPosition(0);
        SetSlideRotatorArmToPosition(0);
        SetClawPosition(Gericka_Hardware.CLAW_MIN_POS);
        SetSwiperPosition(Gericka_Hardware.SWIPER_MAX_POS);
        SetSwiper2Position(Gericka_Hardware.SWIPER2_MAX_POS);
    }

    public void SpecimenDeliverLow(){
        SetIntakeBoxRotatorPosition(0.455); //0.44
        SetSmallArmHangerPosition(0.6); //0.4 //0.65
    }

    // ************************************
    // Hanging Combo Moves
    // ************************************
    public void HooksReleased(){
        lock_position = 1.0;
        lockServo1.setPosition(lock_position);
        lockServo2.setPosition(lock_position);
    }
    public void HooksLocked(){
        lock_position = 0.4;
        lockServo1.setPosition(lock_position);
        lockServo2.setPosition(lock_position);
    }
    //public boolean GetLockPosition() {
    //    return (lock_position == 0.0);
    //}


    public void PreHangRobot(){
        SetSlideRotatorArmToPosition(GetRotatorArmTicksFromDegrees(34.37));
        SpecialSleep(2000);
        SetIntakeBoxRotatorPosition(INTAKE_STAR_BOX_ROTATOR_MAX_POS);
        SetSmallArmHangerPosition(1);
        SpecialSleep(2000);
        SetSlideRotatorArmToPosition(SLIDE_ARM_ROTATOR_MAX_POS);
        SpecialSleep(2000);
        SetSlideToPosition(2415); //3049
    }
    public void ReadyHangRobot(){
        SetSlideRotatorArmToPosition(GetRotatorArmTicksFromDegrees(92));
        SpecialSleep(500);
        SetIntakeBoxRotatorPosition(0.33);
        SetSmallArmHangerPosition(0.79);
        SetClawPosition(1);
        SetSlideToPosition(2693); //1547
    }

    int stepCounter = 0;
    public void level3Ascent() {
        stepCounter = 0;
        int targetAngle = 92;

        boolean originalPidf = pidfSlidesEnabled;
        pidfSlidesEnabled = false;

        // Initialize the Blinkin at the start to be blue
        Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        blinkinLedDriver.setPattern(Blinken_pattern);

        // Initialize the hanger arms and urchin intake go to safe place
        SetIntakeBoxRotatorPosition(0.33);
        SetSmallArmHangerPosition(0.79);
        SpecialSleep(200);

        double timeout = opMode.getRuntime() + 4.0;

        while (stepCounter <= 14) {
            // Continually Hold the arms at 90 degrees from the floor until step 6 (then will clam shell close)
            if (stepCounter < 10) {
                SetSlideRotatorArmToPosition(findRealArmAngle((targetAngle)));
            }

            // *******************************************
            // *** STEP 0 Raises arms to 85 degrees (reference to floor tiles)
            // *** and puts the hanger arms in a safe spot for climbing
            // ***  is Blue while in this step, goes yellow when done
            // *******************************************
            if (stepCounter == 0) {
                if (leftSlideArmRotatorMotor.getCurrentPosition() >= findRealArmAngle(70) && intakeBoxRotatorPosition == 0.33 && smallArmHangerRightPosition == 0.79) {
                    stepCounter++;
                    timeout = opMode.getRuntime() + 5.0;
                } else {
                    targetAngle = 75;
                    SetIntakeBoxRotatorPosition(0.33);
                    SetSmallArmHangerPosition(0.79);
                    SetClawPosition(1);
                    if (leftSlideArmRotatorMotor.getCurrentPosition() >= findRealArmAngle(70) || opMode.getRuntime() > timeout) {
                        Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                        blinkinLedDriver.setPattern(Blinken_pattern);
                        timeout = opMode.getRuntime() + 5.0;
                        stepCounter++;
                    }
                }
            }

            // *******************************************
            // *** STEP 1 Arms stay at 85 degrees and
            // *** the slides go out/up, bringing the two
            // *** hooks above the lower bar
            // *** Blinkin is Yellow in this step, goes orange when done
            // *******************************************
            else if (stepCounter == 1) {
                if (slideLeft.getCurrentPosition() >= 2614) {
                    stepCounter++;
                    timeout = opMode.getRuntime() + 5.0;
                }
                else
                {
                    SetSlideToPosition(2693); //3400
                    targetAngle = 75;
                    if (slideLeft.getCurrentPosition() >= 2614 || opMode.getRuntime() > timeout) { //3300
                        Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                        blinkinLedDriver.setPattern(Blinken_pattern);
                        timeout = opMode.getRuntime() + 5.0;
                        stepCounter++;
                    }
                }
            }

            // *******************************************
            // *** STEP 2 Arms rotates to 90 degrees to the floor
            // *** bringing the green hooks and the dolphin fins
            // *** above the middle bar
            // *** Blinkin is orange in this step, goes RED_ORANGE when done
            // *******************************************
            else if (stepCounter == 2) {
                targetAngle = 92;
                SetSlideToPosition(2693); //3400
                if ((slideLeft.getCurrentPosition() >= 2614 && //3300
                        leftSlideArmRotatorMotor.getCurrentPosition() > findRealArmAngle((88)) )
                        || opMode.getRuntime() > timeout) {
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() + 1.0;
                    stepCounter++;
                }
            }

            else if (stepCounter == 3) {
                if (opMode.getRuntime() > timeout) {
                    timeout = opMode.getRuntime() + 3.0;
                    stepCounter++;
                }
            }

            // *******************************************
            // *** STEP 3 Arms stay at 90 degrees and
            // *** the slides come back down to put the
            // *** weight on the dolphin fins.
            // *** as weight transfers to fins, the rotator arm
            // *** has to keep increasing the angle to maintain
            // *** 90 degrees to the floor
            // *** This achieves a LEVEL-2 HANG once the robot is off the floor
            // *** Robot should look very open, just touching the bottom base bar but not the floor
            // *** Blinkin is RED_Orange in this step, goes Red when done
            // *******************************************
            else if (stepCounter == 4) {
                targetAngle = 70;
                SetSlideToPosition(0);
                if (slideLeft.getCurrentPosition() <= 20) { // || opMode.getRuntime() > timeout) {
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() + 5.0;
                    stepCounter++;
                }
            }

            else if (stepCounter == 5) {
                targetAngle = 70;
                SetSlideToPosition(341); //430
                if (slideLeft.getCurrentPosition() >= 337) { // || opMode.getRuntime() > timeout) { //425
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() + 5.0;
                    stepCounter++;
                }
            }

            // *******************************************
            // *** STEP 4 Arms reduce to 75 degree angle to the floor
            // *** to allow the green hooks to go beyond the top bar
            // *** without getting caught on the bar and
            // *** the slides extend up / out
            // *** Blinkin is Red in this step, goes HOT_PINK when done
            // *******************************************
            else if (stepCounter == 6) {
                targetAngle = 75;
                SetSlideToPosition(4039); //5100
                if ((leftSlideArmRotatorMotor.getCurrentPosition() >= findRealArmAngle(72) &&
                        leftSlideArmRotatorMotor.getCurrentPosition() < findRealArmAngle(78) &&
                        slideLeft.getCurrentPosition() >= 4039) //5100
                        || opMode.getRuntime() > timeout) {
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() + 3.0;
                    stepCounter++;
                }
            }
            // *******************************************
            // *** STEP 5 Arms returns to 90 degrees to the floor
            // *** bringing the green hooks directly above the
            // *** top bar, robot is very open touching multiple bars
            // *** Blinkin is Pink in this step, goes Aqua when done
            // *******************************************
            else if (stepCounter == 7) {
                armRotatorOverride = true;
                targetAngle = 95;
                SetSlideToPosition(3881); //4900
                if ((slideLeft.getCurrentPosition() >= 3841 && //4850
                        leftSlideArmRotatorMotor.getCurrentPosition() >= findRealArmAngle((88)) )
                    //|| opMode.getRuntime() > timeout
                ) {
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() + 1.0;
                    stepCounter++;
                }
            }

            else if (stepCounter == 8) {
                if (opMode.getRuntime() > timeout) {
                    timeout = opMode.getRuntime() + 1.5; //3.0
                    stepCounter++;
                }
            }
            // *******************************************
            // *** STEP 6 Arms stay at 90 degrees to the floor and
            // *** the slides come down to hook the green
            // *** hooks on the HIGH bar, and bring weight off of the dolphin fins
            // *** But the slides stay extended enough to keep the robot
            // *** touching both horizontal bars and probably still touching the bottom bar (but still off the floor)
            // *** Blinkin is Aqua in this step, goes GREEN when done
            // *******************************************
            else if (stepCounter == 9) {
                targetAngle = 95;
                SetSlideToPosition(3485); //4400
                if (slideLeft.getCurrentPosition() <= 3490 || opMode.getRuntime() > timeout) { //4400
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() + 1.5; //3.0
                    stepCounter++;
                }
            }

            // *******************************************
            // *** STEP 7 Arms stay at 90 degrees to the floor
            // *** Because it is pressed against both horizontal bars,
            // *** the base of the robot clam-shells closed
            // *** showing the SHEAR FORCE bottom to the crowd!!!
            // *** Blinkin is GREEN and stays Green from here on out to look cool
            // *******************************************
            else if (stepCounter == 10)
            {
                // CLAM SHELL CLOSED
                SetSlideRotatorArmToPosition(0);
                if (GetSlidesLimitSwitchPressed() || opMode.getRuntime() > timeout) {
                    timeout = opMode.getRuntime() + 0.25; //0.5
                    stepCounter++;
                }
            }

            // *******************************************
            // *** STEP 8 Arms stay at 90 degrees to the floor
            // *** Because it is pressed against both horizontal bars, (but no longer the bottom base bar)
            // *** Blinkin STAYS GREEN
            // *** Lock pin servos lock the base of the robot in the clam shell position
            // *******************************************
            else if (stepCounter == 11)
            {
                HooksLocked();
                if (opMode.getRuntime() > timeout) {
                    timeout = opMode.getRuntime() + 1.5; //3.0
                    stepCounter++;
                }
            }

            // *******************************************
            // *** STEP 9
            // *** Arm Rotator power removed (lock pins holding now)
            // *** Blinkin should stay GREEN here
            // *** Slides come in (but not all the way,
            // *** just enough to balance the weight of the back of the robot)
            // *** As slides comes in the robot comes lose from
            // *** the lower hang bar and swings to a horizontal state
            // *******************************************
            else if (stepCounter == 12)
            {
                // Remove power from arm rotators when locks are engaged
                SetSlideRotatorToPowerMode(0);
                SetSlideToPosition(1030); //1300
                if (slideLeft.getCurrentPosition() <= 1045 || opMode.getRuntime() > timeout) { //1320
                    timeout = opMode.getRuntime() + 1.0;
                    stepCounter++;
                }
            }

            else if (stepCounter == 13)
            {
                if (opMode.getRuntime() > timeout) {
                    timeout = opMode.getRuntime() + 3.0;
                    stepCounter++;
                }
            }

            else if (stepCounter == 14)
            {
                SetSlideToPosition(832); //1050
            }

            if (opMode.gamepad1.share) {
                armRotatorOverride = false;
                break;
            }



            ShowTelemetry();
            SpecialSleep(50);
        } // END while loop
        pidfSlidesEnabled = originalPidf;
    }


    public void level2Ascent() {
        stepCounter = 0;
        int targetAngle = 75;

        // Initialize the Blinkin at the start to be blue
        Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        blinkinLedDriver.setPattern(Blinken_pattern);

        // Initialize the hanger arms and urchin intake go to safe place
        SetIntakeBoxRotatorPosition(0.33);
        SetSmallArmHangerPosition(0.79);
        SpecialSleep(200);

        double timeout = opMode.getRuntime() + 4.0;

        while (stepCounter <= 9) {
            // Continually Hold the arms at 90 degrees from the floor until step 6 (then will clam shell close)
            if (stepCounter < 4) {
                SetSlideRotatorArmToPosition(findRealArmAngle((targetAngle)));
            }

            // *******************************************
            // *** STEP 0 Raises arms to 85 degrees (reference to floor tiles)
            // *** and puts the hanger arms in a safe spot for climbing
            // ***  is Blue while in this step, goes yellow when done
            // *******************************************
            if (stepCounter == 0) {
                targetAngle = 75;
                SetIntakeBoxRotatorPosition(0.33);
                SetSmallArmHangerPosition(0.79);
                SetClawPosition(1);
                if (leftSlideArmRotatorMotor.getCurrentPosition() >= findRealArmAngle(70) || opMode.getRuntime() > timeout) {
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() +  5.0;
                    stepCounter++;
                }
            }

            // *******************************************
            // *** STEP 1 Arms stay at 85 degrees and
            // *** the slides go out/up, bringing the two
            // *** hooks above the lower bar
            // *** Blinkin is Yellow in this step, goes orange when done
            // *******************************************
            else if (stepCounter == 1) {
                SetSlideToPosition(2693); //3400
                targetAngle = 75;
                if (slideLeft.getCurrentPosition() >= 3300 || opMode.getRuntime() > timeout) {
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() + 5.0;
                    stepCounter++;
                }
            }

            // *******************************************
            // *** STEP 2 Arms rotates to 90 degrees to the floor
            // *** bringing the green hooks and the dolphin fins
            // *** above the middle bar
            // *** Blinkin is orange in this step, goes RED_ORANGE when done
            // *******************************************
            else if (stepCounter == 2) {
                targetAngle = 90;
                SetSlideToPosition(2693); //3400
                if ((slideLeft.getCurrentPosition() >= 3300 &&
                        leftSlideArmRotatorMotor.getCurrentPosition() > findRealArmAngle((88)) )
                        || opMode.getRuntime() > timeout) {
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() + 1.0;
                    stepCounter++;
                }
            }

            else if (stepCounter == 3) {
                if (opMode.getRuntime() > timeout) {
                    timeout = opMode.getRuntime() + 3.0;
                    stepCounter++;
                }
            }

            // *******************************************
            // *** STEP 3 Arms stay at 90 degrees and
            // *** the slides come back down to put the
            // *** weight on the dolphin fins.
            // *** as weight transfers to fins, the rotator arm
            // *** has to keep increasing the angle to maintain
            // *** 90 degrees to the floor
            // *** This achieves a LEVEL-2 HANG once the robot is off the floor
            // *** Robot should look very open, just touching the bottom base bar but not the floor
            // *** Blinkin is RED_Orange in this step, goes Red when done
            // *******************************************
            else if (stepCounter == 4) {
                armRotatorOverride = true;
                SetSlideRotatorArmToPosition(findRealArmAngle(90));
                SetSlideRotatorArmToHoldCurrentPosition();
                //targetAngle = 90; //70
                SetSlideToPosition(832); //1050
                if (slideLeft.getCurrentPosition() <= 20) { // || opMode.getRuntime() > timeout) {
                    Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    blinkinLedDriver.setPattern(Blinken_pattern);
                    timeout = opMode.getRuntime() + 5.0;
                    stepCounter++;
                }
            }

            else if (stepCounter == 5) {
                armRotatorOverride = true;
                SetSlideRotatorArmToPosition(findRealArmAngle(20));
                SetSlideRotatorArmToHoldCurrentPosition();
            }

            if (opMode.gamepad1.share) {
                armRotatorOverride = false;
                break;
            }



            ShowTelemetry();
            SpecialSleep(50);
        } // END while loop
    }


    // ************************************
    // High Specimen Delivery Combo Moves
    // ************************************
    public void SpecimenPickupFromWall() {
        SpecimenPickupFromWallServoPosition();
        SetClawPosition(CLAW_MIN_POS);

        double timeout = opMode.getRuntime() + 0.5;
        SetSlideToPosition(0);
        while (!GetSlidesLimitSwitchPressed() && opMode.getRuntime() < timeout) {
            SpecialSleep(50);
        }

        SetSlideRotatorArmToPosition(0);
    }
    public void SpecimenPickupFromWallServoPosition(){
        SetIntakeBoxRotatorPosition(0.93); //0.875     ORIGINALLY 0.89
        SetSmallArmHangerPosition(0.35); //.15 //0.4
    }
    public void RemoveFromWall(){
        RemoveFromWallServoPosition();

        double timeout = opMode.getRuntime() + 0.5;
        SetSlideToPosition(0);
        while (!GetSlidesLimitSwitchPressed() && opMode.getRuntime() < timeout) {
            SpecialSleep(50);
        }
        SetSlideRotatorArmToPosition(0);
    }
    public void Stow(){
        //RemoveFromWallServoPosition();
        SetIntakeBoxRotatorPosition(0.305); //0.875 //0.96  //0.2
        SetSmallArmHangerPosition(0.2); //0 //0.25

        double timeout = opMode.getRuntime() + 2.0;
        SetSlideToPosition(0);
        while (!GetSlidesLimitSwitchPressed() && opMode.getRuntime() < timeout) {
            SpecialSleep(50);
            if (GetSlidesLimitSwitchPressed()) {
                ResetSlidesToZero();
            }
        }

        SetSlideRotatorArmToPosition(0);
    }
    public void RemoveFromWallServoPosition() {
        SetIntakeBoxRotatorPosition(0.89); //0.875
        SetSmallArmHangerPosition(0.2); //0 //0.25
    }
    public void SpecimenDeliverHighChamberAlternate(){
        SetIntakeBoxRotatorPosition(0.948); //0.82  //0.905  //0.908
        SetSmallArmHangerPosition(0.22); //0 //0.25 //.20
        SetSlideToPosition(1063);  //1240  //740 //1342
        SetSlideRotatorArmToPosition(GetRotatorArmTicksFromDegrees(72)); //75.55 //1259  //85 // most recent: 70.68
    }
    public void SpecimenDeliverHighChamberFinishingMove(){
        SetIntakeBoxRotatorPosition(0.948); //0.908 //0.945
        SetSmallArmHangerPosition(0.22); //0 //0.25 //0.2
        SetSlideToPosition(2297); //00  //2350  //1750 //2900
        SetSlideRotatorArmToPosition(GetRotatorArmTicksFromDegrees(72)); //642 //most recent: 70.68
    }
    public void UrchinPickupFromWall(){
            UrchinPickupFromWallServoPosition();
            SetUrchinServoPosition(URCHIN_SERVO_MIN_POS);

            double timeout = opMode.getRuntime() + 2.0;
            SetSlideToPosition(0);
            while (!GetSlidesLimitSwitchPressed() && opMode.getRuntime() < timeout) {
                SpecialSleep(50);
            }
            if (GetSlidesLimitSwitchPressed()) {
                ResetSlidesToZero();
            }

            SetSlideRotatorArmToPosition(0);
        }
        public void UrchinPickupFromWallServoPosition(){
            SetIntakeBoxRotatorPosition(0.435); //0.59 //0.535
            SetSmallArmHangerPosition(0.57);
        }
        public void UrchinRemoveFromWall(){
            SetUrchinServoPosition(URCHIN_SERVO_MAX_POS);
            SpecialSleep(300);
            UrchinRemoveFromWallServoPosition();

        double timeout = opMode.getRuntime() + 0.5;
        SetSlideToPosition(0);
        while (!GetSlidesLimitSwitchPressed() && opMode.getRuntime() < timeout) {
            SpecialSleep(50);
        }
        SetSlideRotatorArmToPosition(0);
    }
    public void UrchinRemoveFromWallServoPosition(){
        SetIntakeBoxRotatorPosition(0.53);
        SetSmallArmHangerPosition(0.2);
    }
    public void UrchinDeliverHighChamberAlternate(){
        SetIntakeBoxRotatorPosition(0.345); //0.82  //0.905 //0.49 //0.575
        SetSmallArmHangerPosition(.20); //0 //0.25
        SetSlideToPosition(1605);  //1240  //740 //1240 //2027
        SetSlideRotatorArmToPosition(GetRotatorArmTicksFromDegrees(75.55));
    }
    public void UrchinDeliverHighChamberFinishingMove(){
        SetIntakeBoxRotatorPosition(0.345); //0.945 //0.365 //0.45
        SetSmallArmHangerPosition(0.2); //0 //0.25
        SetSlideRotatorArmToPosition(GetRotatorArmTicksFromDegrees(76.08)); //642 //710
        SetSlideToPosition(2900); //00  //2350  //1750 //2150 //3662
    }

    // ************************************
    // High Basket Delivery Combo Moves
    // ************************************

    // New method
    public void SampleUrchinFloorPickup(){
        // if the rotator arm is raised for some reason
        if (GetRotatorLeftArmCurrentPosition() > 20)
        {
            SetSlideToPosition(0);
            double timeout = opMode.getRuntime() + 4.0;
            while (slideLeft.getCurrentPosition() > 400 && opMode.getRuntime() < timeout)
            {
                SpecialSleep(50);
            }
            SetSlideRotatorArmToPosition(0);
            timeout = opMode.getRuntime() + 2.0;
            while (leftSlideArmRotatorMotor.getCurrentPosition() > findRealArmAngle(5) && opMode.getRuntime() < timeout)
            {
                SpecialSleep(50);
            }

        }
        // if the slides need to be moved out in front of the robot still (doesn't waste time if already there)
        if (GetSlideLeftCurrentPosition() < 1346 ) { //1700
            // Move slides a little in front of robot
            SampleUrchinFloorPickup_SlidePosition();
            SpecialSleep(300);
        }
        // if the urchin needs to be positioned still (doesn't waste time if already there)
        if (intakeBoxRotatorPosition != 0.2 || smallArmHangerLeftPosition != 0.56 ) {
            // Open the urchin and position to be ready to intake
            //SampleUrchinFloorPickup_UrchinReadyPosition();
            SampleUrchinLimelightViewPosition();
            SpecialSleep(400);
        }
        // if need to open the Urchin
        if (urchinServo_position != 0) {
            SetUrchinServoPosition(0);
            SpecialSleep(200);
        }

        // if limelight is enabled
        if (limelightEnabled)
        {
            // center up the robot on the sample using the limelight
            AlignOnFloorSample();

        }
    }


    public void SampleUrchinLimelightViewPosition()
    {
        SetIntakeBoxRotatorPosition(0.2);
        SetSmallArmHangerPosition(0.56);
    }

    public void SampleUrchinFloorJam(){
        // if the slides need to be moved out in front of the robot still (doesn't waste time if already there)
        if (!(GetRotatorLeftArmCurrentPosition() <= 20 && GetSlideLeftCurrentPosition() >= 1346 )) { //1700
            // Move slides a little in front of robot
            SampleUrchinFloorPickup_SlidePosition();
            SpecialSleep(300);
        }

        // if need to close the Urchin
        if (urchinServo_position != 1) {
            SetUrchinServoPosition(1);
            SpecialSleep(200);
        }

        // move the urchin position to jam into the floor
        SampleUrchinFloorPickup_UrchinJamReadyPosition();
    }
    public void SampleUrchinFloorPickup_SlidePosition() {
        // Move slides a little in front of robot
        SetSlideRotatorArmToPosition(0);
        SetSlideToPosition(1426);  // 1945 //1800
    }
    public void SampleUrchinFloorPickup_UrchinReadyPosition() {
        // position the urchin to be ready to intake
        SetIntakeBoxRotatorPosition(0.355);
        SetSmallArmHangerPosition(0.7); //.15 //0.80
        SetSlideRotatorArmToPosition(0);
    }
    public void SampleUrchinFloorPickup_UrchinJamReadyPosition() {
        // position the urchin to be ready to intake
        SetIntakeBoxRotatorPosition(0.51); //0.47
        SetSmallArmHangerPosition(0.83); //0.87 //0.15 //0.80 //0.81
        SetSlideRotatorArmToPosition(0);
    }
    public void SampleUrchinFloorPickupFinishingMove(){
        // Lower the urchin to be closer to the floor
        SampleUrchinFloorPickupFinishingMove_UrchinGrabPosition();
        SpecialSleep(100);

        // Close the urchin to grab the specimen
        SetUrchinServoPosition(1);
        SpecialSleep( 400); //300

        // Raise the Urchin back up to ready position to assess if succeeded
        if (limelightEnabled)
        {
            SampleUrchinLimelightViewPosition();
        }
        else {
            SampleUrchinFloorPickup_UrchinReadyPosition();
        }
    }
    public void SampleUrchinFloorPickupFinishingMove_UrchinGrabPosition() {
        // Lower the urchin to be closer to the floor
        SetIntakeBoxRotatorPosition(0.485);   //0.485 //0.525 //0
        SetSmallArmHangerPosition(0.85); //.15 //0.80 //0.8
        SetSlideRotatorArmToPosition(0);
    }

    public void InspectionLowForward(){
        // if the rotator arm is raised
        if (GetRotatorLeftArmCurrentPosition() > GetRotatorArmTicksFromDegrees(10))
        {
            // if the rotator arm is raised a lot (be more careful on lowering it)
            if (GetRotatorLeftArmCurrentPosition() > GetRotatorArmTicksFromDegrees(70))
            {
                // Rotate urchin back away from the basket
                BasketHighFinishingMove_UrchinSafeToLowerPosition();
                SpecialSleep(2000);
                // Rotate arms a little away from basket and lower slides to zero
                BasketHighFinishingMove_ArmSafeToLowerPosition();
                SpecialSleep(2000); //400
            }
            Stow();
        }
        SetSlideToPosition(SLIDE_ARM_MAX_HORIZONTAL_POS);
        SetIntakeBoxRotatorPosition(0.96); //0.875
        SetSmallArmHangerPosition(0.38);
        SetClawPosition(CLAW_MIN_POS);
    }

    public void InspectionHighPos(){
        // if the slides are currently horizontally extended, but the rotator arm is not raised yet
        if (GetRotatorLeftArmCurrentPosition() <= GetRotatorArmTicksFromDegrees(30) && GetSlideLeftCurrentPosition() >= 1346 ) { //1700
            // Stow the urchin before raising the arms and reraising the slides
            Stow();
        }

        BasketHighFinishingMove_UrchinDeliverPosition();
        BasketHighFinishingMove_SlidesPosition();
    }

    public void BasketHigh(){
        //STEP ONE
        BasketHighFinishingMove_UrchinSafeToLowerPosition();

        SetIntakeBoxRotatorPosition(0.865); //0.85
      //  SetSmallArmHangerPosition(1.0); //.8 //1.05
        // if in a horizontal arm position, then should do a reset on the slides, first before raising the arm to keep from tipping over
        if (leftSlideArmRotatorMotor.getCurrentPosition() < GetRotatorArmTicksFromDegrees(20)) {
            double timeout = opMode.getRuntime() + 1.5; //2.0
            SetSlideToPosition(0);
            while (!GetSlidesLimitSwitchPressed() && opMode.getRuntime() < timeout) {
                SpecialSleep(50);
            }
            if (GetSlidesLimitSwitchPressed()) {
                ResetSlidesToZero();
            }
        }

        SetSlideRotatorArmToPosition(GetRotatorArmTicksFromDegrees(85.13)); //8008, 450
        // wait for the rotators to move to vertical before raising slides
        //SpecialSleep(2000);
        //SetSlideToPosition(5145); //2320 //6496
    }

    public void BasketHighFinishingMove(){

        // if driver forgot to prepare and is trying to go straight up from a horizontal extended position
        if (leftSlideArmRotatorMotor.getCurrentPosition() < GetRotatorArmTicksFromDegrees(70))
        {
            BasketHigh();
        }
        // Raise slides to high basket height
        BasketHighFinishingMove_SlidesPosition();
        double timeout = opMode.getRuntime() + 4.0;
        while (slideLeft.getCurrentPosition() < 5530 && opMode.getRuntime() < timeout)
        {
            SpecialSleep(50);
        }
        //SpecialSleep(2500); //2000 //4000

        // Rotate urchin to align above basket
        BasketHighFinishingMove_UrchinDeliverPosition();
        SpecialSleep(200); //200
        // Release the sample from the urchin
        SetUrchinServoPosition(0);
        SpecialSleep(400);
        // Rotate urchin back away from the basket
        BasketHighFinishingMove_UrchinSafeToLowerPosition();
        SpecialSleep(200);
        // Rotate arms a little away from basket and lower slides to zero
        BasketHighFinishingMove_ArmSafeToLowerPosition();
        SpecialSleep(1500); //200
        SetSlideToPosition(0);
    }
    public void BasketHighFinishingMove_SlidesPosition(){
        SetSlideToPosition(4381); //6560 //5532
    }
    public void BasketHighFinishingMove_UrchinDeliverPosition() {
        SetIntakeBoxRotatorPosition(0.705);
        SetSmallArmHangerPosition(0.52);  //1.0
        SetSlideRotatorArmToPosition(GetRotatorArmTicksFromDegrees(85.13));
    }
    public void BasketHighFinishingMove_UrchinSafeToLowerPosition(){
        SetSmallArmHangerPosition(.64); //1    .54 original
    }
    public void BasketHighFinishingMove_ArmSafeToLowerPosition(){
        SetSlideRotatorArmToPosition(GetRotatorArmTicksFromDegrees(74.49));
    }

    // hanger position 0.8
    // .15 right
    //slides - -2320
    //rotator 8008
    //0.85 IntakeStarRotator, hanger position (0.15/0.8)

    // *********************************************************
    // ****       Green Intake Box Controls                 ****
    // *********************************************************

    public void SetIntakeBoxRotatorPosition(double position)
    {
        intakeBoxRotatorPosition = position;
        // Limit the range to valid values
        intakeBoxRotatorPosition = Math.min(intakeBoxRotatorPosition, INTAKE_STAR_BOX_ROTATOR_MAX_POS);
        intakeBoxRotatorPosition = Math.max(intakeBoxRotatorPosition, INTAKE_STAR_BOX_ROTATOR_MIN_POS);

        intakeBoxRotaterServo.setPosition(intakeBoxRotatorPosition);
    }

    public void SetIntakeBoxRotatorIncrementUp()
    {
        intakeBoxRotatorPosition += INTAKE_STAR_BOX_ROTATOR_INCREMENT;
        SetIntakeBoxRotatorPosition(intakeBoxRotatorPosition);
    }

    public void SetIntakeBoxRotatorDecrementDown()
    {
        intakeBoxRotatorPosition -= INTAKE_STAR_BOX_ROTATOR_INCREMENT;
        SetIntakeBoxRotatorPosition(intakeBoxRotatorPosition);
    }
// *********************************************************
    // ****       Urchin Controls                     ****
    // *********************************************************

    public void SetUrchinServoPosition(double position)
    {
        urchinServo_position = position;
        // Limit the range to valid values
        urchinServo_position = Math.min(urchinServo_position, URCHIN_SERVO_MAX_POS);
        urchinServo_position = Math.max(urchinServo_position, URCHIN_SERVO_MIN_POS);

        urchinServo.setPosition(urchinServo_position);
    }


    // *********************************************************
    // ****       Intake Stars Controls                     ****
    // *********************************************************
/*
    public void SetIntakeStarPower(double power)
    {
        intakeStarPower = power;
        intakeStarServo.setPower(intakeStarPower);
        if (power > 0.0)
        {
            intakeStarLastForward = true;
        }
        else if (power < 0.0)
        {
            intakeStarLastForward = false;
        }

    }

    public void CycleIntakeStarMode() {

        if (intakeStarPower != 0.0) {
            SetIntakeStarPower(0);
        } else if(intakeStarLastForward) {
            SetIntakeStarPower(-1.0);
        }
        else {
            SetIntakeStarPower(1.0);

        }


    }

 */

    // *********************************************************
    // ****       CLAW Controls                             ****
    // *********************************************************

    public void SetClawPosition(double position)
    {
        claw_position = position;
        // Limit the range to valid values
        claw_position = Math.min(claw_position, CLAW_MAX_POS);
        claw_position = Math.max(claw_position, CLAW_MIN_POS);

        clawServo.setPosition(claw_position);
    }

    public void SetSwiperPosition(double position)
    {
        double swiper_position = position;
        // Limit the range to valid values
        swiper_position = Math.min(swiper_position, SWIPER_MAX_POS);
        swiper_position = Math.max(swiper_position, SWIPER_MIN_POS);

        swiperServo.setPosition(swiper_position);
    }

    public void SetSwiper2Position(double position)
    {
        double swiper2_position = position;
        // Limit the range to valid values
        swiper2_position = Math.min(swiper2_position, SWIPER2_MAX_POS);
        swiper2_position = Math.max(swiper2_position, SWIPER2_MIN_POS);


        swiper2.setPosition(swiper2_position);
    }


    // **********************************************************
    // ****       Small Arm (Hangers) Controls              ****
    // *********************************************************

    public void SetSmallArmHangerIncrementUp()
    {
        smallArmHangerRightPosition += SMALL_ARM_HANGER_INCREMENT;
        SetSmallArmHangerPosition(smallArmHangerRightPosition);
    }

    public void SetSmallArmHangerDecrementDown()
    {
        smallArmHangerRightPosition -= SMALL_ARM_HANGER_INCREMENT;
        SetSmallArmHangerPosition(smallArmHangerRightPosition);
    }

    public void SetSmallArmHangerPosition(double position)
    {
        smallArmHangerRightPosition = position;
        // Limit the range to valid values
        smallArmHangerRightPosition = Math.min(smallArmHangerRightPosition, SMALL_ARM_HANGER_MAX_POS);
        smallArmHangerRightPosition = Math.max(smallArmHangerRightPosition, SMALL_ARM_HANGER_MIN_POS);

        smallArmHangerLeftPosition = SMALL_ARM_HANGER_MAX_POS - position;

        smallArmHangerLeftServo.setPosition(smallArmHangerLeftPosition);
        smallArmHangerRightServo.setPosition(smallArmHangerRightPosition);
    }

    // *********************************************************
    // ****       Slide Arm Controls                ****
    // *********************************************************

    public boolean GetSlidesLimitSwitchPressed(){
        return (!touchSensorSlideLeft.isPressed()) && (!touchSensorSlideRight.isPressed());
    }

    // TODO -- need to determine other rotator arm positions to limit than just zero
    public void Slides_Horizontal_MAX_Limit(){
        // if the rotator arms are in a horizontal orientation
        // AND the slides are starting to go past the max horizontal position
        if ((GetRotatorLeftArmCurrentPosition() <= findRealArmAngle(45)) && (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS)) {
        //if ((GetRotatorRightArmCurrentPosition() <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES || GetRotatorLeftArmCurrentPosition() <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES) &&
        //        (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS || slideRight.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS)){
            // if slides are in power mode
            if (!slidesRunningToPosition) {
                // if user is not commanding slides to go in
                if (slidePower >= 0) {
                    // override the command to limit the slides to a max horizontal position
                    SetSlideToPosition(SLIDE_ARM_MAX_HORIZONTAL_POS);
                }
            }
            // else slides are being commanded to a position, but if slides have been commanded past the limit, bring the slides back in
            else if (slidesTargetPosition > SLIDE_ARM_MAX_HORIZONTAL_POS)
            {
                SetSlideToPosition(SLIDE_ARM_MAX_HORIZONTAL_POS);
            }
        }
    }

    public void SetSlidesToPowerMode(double power)
    {
        //TODO Claire changed this
        if(pidfSlidesEnabled) {
            // if rotator arms are in horizontal position, AND slides are at the limit already, AND commanding to get longer
            if ((GetRotatorRightArmCurrentPosition() == 0 || GetRotatorLeftArmCurrentPosition() == 0) &&
                    (power > 0) &&
                    (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS || slideRight.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS)) {
                // override and ignore the bad command by killing power to the slides
                slidePower = 0;
            } else if (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_VERTICAL_POS && power > 0) {
                slidePower = 0;
            } else {

                slidePower = power;
            }
            slidesRunningToPosition = true; //TODO changed
            slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
        } else{
            // if rotator arms are in horizontal position, AND slides are at the limit already, AND commanding to get longer
            if ((GetRotatorRightArmCurrentPosition() == 0 || GetRotatorLeftArmCurrentPosition() == 0) &&
                    (power > 0) &&
                    (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS || slideRight.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS)) {
                // override and ignore the bad command by killing power to the slides
                slidePower = 0;
            } else if (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_VERTICAL_POS && power > 0) {
                slidePower = 0;
            } else {

                slidePower = power;
            }
            slidesRunningToPosition = false;
            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);

        }
    }

    public void ResetSlidesToZero (){
        slidesTargetPosition = 0;
        if(pidfSlidesEnabled) {

            slidesRunningToPosition = false;
            //TODO changed this
            slidePower = 0;
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
            //.SpecialSleep(50);

            slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } else {


            slidesRunningToPosition = false;
            slidePower = 0;
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
            //this.SpecialSleep(50);
            slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void ResetSlidesToZeroNoWait() {
        slidesTargetPosition = 0;
        if(pidfSlidesEnabled) {
            slidesRunningToPosition = false;
            slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);;
            slideLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        }else{
            slidesRunningToPosition = false;
            slidePower = 0;
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
            slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        }


    }

    public void SetSlideExtensionToPositionPIDF(){
        if (pidfSlidesEnabled) {
            LeftSlide_controller.setPID(Kp, Ki, Kd);

            if (slidesTargetPosition <= 0 && GetSlidesLimitSwitchPressed()) {
                if(slideLeft.getCurrentPosition() != 0 || slideRight.getCurrentPosition() != 0){
                    ResetSlidesToZero();
                }
            } else {

                // if the rotator arms are in a horizontal orientation
                // AND the slides are starting to go past the max horizontal position
                if ((GetRotatorLeftArmCurrentPosition() <= findRealArmAngle(45)) && (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS)) {
                    //if ((GetRotatorRightArmCurrentPosition() <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES || GetRotatorLeftArmCurrentPosition() <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES) &&
                    //        (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS || slideRight.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS)){
                    // if slides are in power mode
                    /*
                    if (!slidesRunningToPosition) {
                        // if user is not commanding slides to go in
                        if (slidePower >= 0) {
                            // override the command to limit the slides to a max horizontal position
                            SetSlideToPosition(SLIDE_ARM_MAX_HORIZONTAL_POS);
                        }
                    }

                     */
                    // else slides are being commanded to a position, but if slides have been commanded past the limit, bring the slides back in
                    if (slidesTargetPosition > SLIDE_ARM_MAX_HORIZONTAL_POS && !armRotatorOverride)
                    {
                        SetSlideToPosition(SLIDE_ARM_MAX_HORIZONTAL_POS);
                    }
                }

                double slide_arm_target = slidesTargetPosition;

                RightSlide_controller.setTolerance(Ktolerance); // sets the error in ticks I think that is tolerated > go back to ticks and degrees, plus or minus the tolerance
                LeftSlide_controller.setTolerance(Ktolerance);  //originally both 5.0 which is half a degree    was at 20 (2 degrees)
                Left_controller.atSetPoint();  //enabled in test code
                // Right_controller.atSetPoint();

                // Calculate the next PID value
                int leftSlide_armPos = (slideLeft.getCurrentPosition());  // int leftSlide_armPos = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2;
                double leftSlide_pid = LeftSlide_controller.calculate(leftSlide_armPos, slide_arm_target);

                //int rightSlide_armPos = slideRight.getCurrentPosition();
                //double rightSlide_pid = RightSlide_controller.calculate(rightSlide_armPos, slide_arm_target);

                // Calculate the FeedForward component to adjust the PID by
                double leftSlide_ff = Math.sin(rotator_arm_target_angle) * Kf;
                //double rightSlide_ff = Math.sin(rotator_arm_target_angle) * Kf;

                // Calculate the motor power (PID + FeedForward) component
                double leftSlidePower = leftSlide_pid + leftSlide_ff;
                //double rightSlidePower = rightSlide_pid + rightSlide_ff;

                //Use braking to slow the motor down faster?
                //slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if(slide_arm_target == 0 && slideLeft.getCurrentPosition() < 200){
                    if(touchSensorSlideLeft.isPressed()){
                        if(armRotatorOverride){
                            slideLeft.setPower(-1);

                        }else {
                            slideLeft.setPower(Math.max(-0.1, leftSlidePower));
                        }


                    }
                    if(touchSensorSlideRight.isPressed()){
                        if(armRotatorOverride){
                            slideRight.setPower(-1);
                        }else {
                            slideRight.setPower(Math.max(-0.1, leftSlidePower));
                        }

                    }



                }else {
                    // Send calculated power to motors
                    slideLeft.setPower(leftSlidePower);
                    //changed
                    slideRight.setPower(leftSlidePower);

                }

                slidesRunningToPosition = true;

            }
        }
    }

    public void SetSlideToPosition (int position)
    {
        // verify not needing to limit because of horizontal limits
        /*
        if (((slideArmRotatorTargetPosition <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES) ||
                (!slideArmRotatorRunningToPosition && leftSlideArmRotatorMotor.getCurrentPosition() <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES) ) &&
                (position >= SLIDE_ARM_MAX_HORIZONTAL_POS)) {
            slidesTargetPosition = SLIDE_ARM_MAX_HORIZONTAL_POS;
        }

         */

        slidesTargetPosition = position;
        // Limit the range to valid values
        slidesTargetPosition = Math.min(slidesTargetPosition, SLIDE_ARM_MAX_VERTICAL_POS);
       // slidesTargetPosition = Math.max(slidesTargetPosition, SLIDE_ARM_MIN_POS);

        if (slideArmRotatorTargetPosition <= findRealArmAngle(45.0) &&
                (slidesTargetPosition >= SLIDE_ARM_MAX_HORIZONTAL_POS) && !armRotatorOverride)
        {
            slidesTargetPosition = SLIDE_ARM_MAX_HORIZONTAL_POS;
        }

        slidesRunningToPosition = true;
        slideLeft.setTargetPosition(slidesTargetPosition);
        slideRight.setTargetPosition(slidesTargetPosition);

        if(!pidfSlidesEnabled) {
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slidePower = SLIDES_POS_POWER;
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
        }
    }

    public void SetSlidesToHoldCurrentPosition()
    {
        //if (slidesTargetPosition == 0)
        //{
        //    ResetSlidesToZero();
        //}
        //else {



        if(!pidfSlidesEnabled) {
            //TODO moved into method
            slidesTargetPosition = slideLeft.getCurrentPosition();

            slidePower = 0;
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
            slideLeft.setTargetPosition(slidesTargetPosition);
            slideRight.setTargetPosition(slideRight.getCurrentPosition());
            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slidesRunningToPosition = false;
        }
        //}
      //  slidesRunningToPosition = true; //TODO changed
    }

    public boolean GetSlidesRunningToPosition() { return slidesRunningToPosition; }
    public int GetSlidesTargetPosition() { return slidesTargetPosition; }
    public int GetSlideLeftCurrentPosition() { return slideLeft.getCurrentPosition(); }
    public int GetSlideRightCurrentPosition() { return slideRight.getCurrentPosition(); }

    // *********************************************************
    // ****       Slide Rotator Arm Controls                ****
    // *********************************************************

    public int GetRotatorArmTicksFromDegrees (double target_arm_angle)
    {
        return ((int)(target_arm_angle * ticks_in_degrees));
    }

    public void SetPIDF_Enabled(boolean enabled)
    {
        pidfEnabled = enabled;
    }
    public void SetPIDF_Slides_Enabled(boolean enabled)
    {
        pidfSlidesEnabled = enabled;
    }
    public void SetSlideRotatorArmToPositionPIDF(){
        if (pidfEnabled) {
            //Right_controller.setPID(p,i,d);
            //Left_controller.setPID(p,i,d);

/*          // Multi-threading lock example
            lock.lock();
            try {
                //do stuff here
            } finally {
                lock.unlock();
            }
*/
            //store in geronimo rotator_arm_target_ticks. Call loop again and again. then if and everything else, else



            if (rotator_arm_target_ticks == 0 && GetSlideRotatorArmLimitSwitchPressed()) {
                ResetSlideRotatorArmToZero();
            } else {
                rotator_arm_target_angle = rotator_arm_target_ticks / ticks_in_degrees;

                Right_controller.setTolerance(10.0); // sets the error in ticks I think that is tolerated > go back to ticks and degrees, plus or minus the tolerance
                Left_controller.setTolerance(10.0);  //originally both 5.0 which is half a degree    was at 20 (2 degrees)
                //Left_controller.atSetPoint();
               // Right_controller.atSetPoint();

                // Calculate the next PID value
                int left_armPos = (leftSlideArmRotatorMotor.getCurrentPosition() + rightSlideArmRotatorMotor.getCurrentPosition())/2;
                double left_pid = Left_controller.calculate(left_armPos, rotator_arm_target_ticks);

                int right_armPos = rightSlideArmRotatorMotor.getCurrentPosition();
                double right_pid = Right_controller.calculate(right_armPos, rotator_arm_target_ticks);

                // Calculate the FeedForward component to adjust the PID by
                double left_ff = Math.cos(rotator_arm_target_angle) * f;
                double right_ff = Math.cos(rotator_arm_target_angle) * f;

                // Calculate the motor power (PID + FeedForward) component
                double leftPower = left_pid + left_ff;
                double rightPower = right_pid + right_ff;

                //leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Send calculated power to motors
                leftSlideArmRotatorMotor.setPower(leftPower);
                rightSlideArmRotatorMotor.setPower(leftPower);

            }
        }

    }

    public void SetSlideRotatorToPowerMode(double power)
    {
        slideArmRotatorPower = power;
        //820 to 920 for limit -- SLIDE_ARM_ROTATOR_MAX_POS
        if (slideArmRotatorPower > 0 && leftSlideArmRotatorMotor.getCurrentPosition() >= findRealArmAngle(90)) {
            SetSlideRotatorArmToPosition(SLIDE_ARM_ROTATOR_MAX_POS);
        }
        else {
            slideArmRotatorRunningToPosition = false;
            leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);

            rightSlideArmRotatorMotor.setPower(slideArmRotatorPower);
            leftSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        }

    }

    //public void SetSlideRotatorArmToAngle (int targetAngle)
    //{
    //    SetSlideRotatorArmToPosition(findRealArmAngle((targetAngle)));
    //}
    private boolean armRotatorOverride = false;
    public void SetSlideRotatorArmToPosition(int position)
    {
        slideArmRotatorPower = 1.0;
        if(isRobotLevel() && !armRotatorOverride)
        {
            // if the arm rotator is being commanded down, and the target position to rotate to will be less than 45 degrees
            if(leftSlideArmRotatorMotor.getCurrentPosition() > position || position <= findRealArmAngle(45))
            //if(leftSlideArmRotatorMotor.getCurrentPosition() < position || position < SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES)
            {
                if (GetSlideLeftCurrentPosition() > SLIDE_ARM_MAX_HORIZONTAL_POS)
                {
                    SetSlideToPosition(SLIDE_ARM_MAX_HORIZONTAL_POS);
                }
            }
        }

        slideArmRotatorTargetPosition = position;
        // Limit the range to valid values
        /*
        if (!armRotatorOverride) {
            slideArmRotatorTargetPosition = Math.min(slideArmRotatorTargetPosition, findRealArmAngle(90));
        }
        else {
            slideArmRotatorTargetPosition = Math.min(slideArmRotatorTargetPosition, findRealArmAngle(97));
        }
        */

        slideArmRotatorTargetPosition = Math.min(slideArmRotatorTargetPosition, findRealArmAngle(90));
        slideArmRotatorTargetPosition = Math.max(slideArmRotatorTargetPosition, SLIDE_ARM_ROTATOR_MIN_POS);

        slideArmRotatorRunningToPosition = true;

        if (pidfEnabled)
        {
         //   SetSlideRotatorArmToPositionPIDF(position);
/*          // Multi-threading lock example
            lock.lock();
            try {
                //do stuff here
            } finally {
                lock.unlock();
            }
*/

            rotator_arm_target_ticks = position;
        }
        else {
            rightSlideArmRotatorMotor.setTargetPosition(slideArmRotatorTargetPosition);
            leftSlideArmRotatorMotor.setTargetPosition(slideArmRotatorTargetPosition);
            leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideArmRotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideArmRotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftSlideArmRotatorMotor.setPower(slideArmRotatorPower);
            rightSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        }
    }
    public void ResetSlideRotatorArmToZero(){
        slideArmRotatorPower = 0.0;
        leftSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        rightSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        //this.SpecialSleep(50);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void SetSlideRotatorArmToZero(){
        slideArmRotatorPower = 0.0;
        leftSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        rightSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        //this.SpecialSleep(50);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public boolean GetSlideRotatorArmLimitSwitchPressed(){
        return (!touchSensorSlideArmRotatorRight.isPressed()) || (!touchSensorSlideArmRotatorLeft.isPressed());
    }
    public boolean GetSlideRotatorBothArmLimitSwitchPressed(){
        return (!touchSensorSlideArmRotatorRight.isPressed()) && (!touchSensorSlideArmRotatorLeft.isPressed());
    }
    public void SetSlideRotatorArmToHoldCurrentPosition() {
        if (!pidfEnabled) {


        //if (slideArmRotatorTargetPosition == 0) {
        //    ResetSlideRotatorArmToZero();
        //}
        //else {
        slideArmRotatorPower = SLIDE_ARM_ROTATOR_POWER;
        leftSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        rightSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        leftSlideArmRotatorMotor.setTargetPosition(leftSlideArmRotatorMotor.getCurrentPosition());
        rightSlideArmRotatorMotor.setTargetPosition(leftSlideArmRotatorMotor.getCurrentPosition());
        //}
        slideArmRotatorRunningToPosition = false;
    }
    }
    public boolean GetRotatorArmRunningToPosition()
    {
        return slideArmRotatorRunningToPosition;
    }
    public int GetRotatorArmTargetPosition()
    {
        return slideArmRotatorTargetPosition;
    }
    public int GetRotatorLeftArmCurrentPosition()
    {
        return (leftSlideArmRotatorMotor.getCurrentPosition());
    }
    public int GetRotatorRightArmCurrentPosition()
    {
        return (rightSlideArmRotatorMotor.getCurrentPosition());
    }

    public void ShowTelemetry(){
        double[] col = getColorTxTy();                 // our helper returns {tx,ty} or {NaN,NaN}


        // after you display color tx/ty…
        if (!Double.isNaN(col[0])) {
            opMode.telemetry.addData("Color-tx / ty (deg)", "%.1f , %.1f", col[0], col[1]);

            double[] sco = getStrafeOffsetInInchesColor();
            opMode.telemetry.addData("Color-based strafe X/Y (in)", "%.2f , %.2f", sco[0], sco[1]);
        } else {
            opMode.telemetry.addData("Color-tx / ty (deg)", "NO COLOR TARGET");
        }

        opMode.telemetry.addData(
                "align finalX / finalY (in)",
                "%.2f , %.2f",
                finalX, finalY
        );



        opMode.telemetry.addData("Slide Arm Rotator Positions: ", "L: %d, R: %d", leftSlideArmRotatorMotor.getCurrentPosition(), rightSlideArmRotatorMotor.getCurrentPosition());
        opMode.telemetry.addData("slides Position ", "L: %d, R: %d", slideLeft.getCurrentPosition(), slideRight.getCurrentPosition());
        opMode.telemetry.addData("stepCounter: ", stepCounter);

        opMode.telemetry.addData("Limelight Enabled: " , limelightEnabled);
        opMode.telemetry.addData("Limelight Target: " , limelight_targetImageName);
        opMode.telemetry.addData("Limelight Connected: " , limelightbox.isConnected());
        opMode.telemetry.addData("Limelight IsRunning: " , limelightbox.isRunning());
        opMode.telemetry.addData("Limelight Version: " , limelightbox.getVersion());
        //opMode.telemetry.addData("FindAngleToTargetImageMethod:", FindAlignAngleToTargetImage("red"));
        //opMode.telemetry.addData("Limelight OffSet (x,y) inches no correction:" ,"R: %.2f, L: %.2f" ,GetStrafeOffsetInInches(limelight_targetImageName)[0], GetStrafeOffsetInInches(limelight_targetImageName)[1]);
        //opMode.telemetry.addData("Limelight Offset (x,y) inches no correction:", "R: %.2f, L: %.2f", GetStrafeOffsetInInches(limelight_targetImageName)[0]+3, GetStrafeOffsetInInches(limelight_targetImageName)[1]);
        opMode.telemetry.addData("Name ", "%s",
                limelight_status.getName());
        opMode.telemetry.addData("LL ", "Temp: %.1fC, CPU: %.1f%%, FPS: %d, RAM: %.1f",
                limelight_status.getTemp(), limelight_status.getCpu(),(int) limelight_status.getFps(), limelight_status.getRam());
        opMode.telemetry.addData("ConnectionInfo: ", limelightbox.getConnectionInfo());
        opMode.telemetry.addData("Pipeline ", "Index: %d, Type: %s",
                limelight_status.getPipelineIndex(), limelight_status.getPipelineType());
        opMode.telemetry.addData("TimeSinceLastUpdate: ", limelightbox.getTimeSinceLastUpdate());
        if(limelight_result != null){
            if (limelight_result.isValid()) {
                opMode.telemetry.addData("tx ", -limelight_result.getTx());
                opMode.telemetry.addData("txnc ", limelight_result.getTxNC());
                opMode.telemetry.addData("ty ", limelight_result.getTy());
                opMode.telemetry.addData("tync ", limelight_result.getTyNC());
            }
            else {
                opMode.telemetry.addData("tx ", "INVALID");
                opMode.telemetry.addData("txnc ", "INVALID");
                opMode.telemetry.addData("ty ", "INVALID");
                opMode.telemetry.addData("tync ", "INVALID");
            }
        }
        else {
            opMode.telemetry.addData("tx ", "NULL");
            opMode.telemetry.addData("txnc ", "NULL");
            opMode.telemetry.addData("ty ", "NULL");
            opMode.telemetry.addData("tync ", "NULL");
        }

        opMode.telemetry.addData("Auto Last Time Left: ", autoTimeLeft);
        opMode.telemetry.addData("imu Heading: ", GetIMU_HeadingInDegrees());

        opMode.telemetry.addData("Claw Position: ", claw_position);
        //opMode.telemetry.addData(">", "Claw - use bumpers for control" );

        opMode.telemetry.addData("Arm Hanger Positions ", "R: %.2f, L: %.2f", smallArmHangerRightPosition, smallArmHangerLeftPosition);
        //opMode.telemetry.addData(">", "Arm Hangers - rightStick X-Axis for control" );

       // opMode.telemetry.addData("Intake Star Power: ", "%.2f, %.2f", intakeStarPower, intakeStarServo.getPower());
        opMode.telemetry.addData("Urchin Position: ", urchinServo_position);
        //opMode.telemetry.addData(">", "Intake Star - use dpad down for control" );


        opMode.telemetry.addData("slides ", "Target: %d, Power: %.2f", slidesTargetPosition, slidePower);
        opMode.telemetry.addData("Slides Left Touched: ", !touchSensorSlideLeft.isPressed());
        opMode.telemetry.addData("Slides Right Touched: ", !touchSensorSlideRight.isPressed());
        opMode.telemetry.addData("Slides in RUN_TO_POSITION? ", slidesRunningToPosition);
        //opMode.telemetry.addData(">", "slides - use leftStick Y for control" );

        opMode.telemetry.addData("PIDF Enabled:", pidfEnabled);
        opMode.telemetry.addData("PIDF Target:", " %.1f (ticks), %.1f (deg) ", rotator_arm_target_ticks, rotator_arm_target_angle);

        opMode.telemetry.addData("Slides Extension PIDF Enabled:", pidfSlidesEnabled);

        opMode.telemetry.addData("Slide Arm Rotator ", "Target: %d, Power: %.2f", slideArmRotatorTargetPosition, slideArmRotatorPower);
        opMode.telemetry.addData("targetPositionIMUARM: " , " %d (ticks), %.1f (deg) ", targetPositionIMUARM_ticks, targetIMU_Degrees);
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        opMode.telemetry.addData("imu roll: ", (imu.getRobotYawPitchRollAngles().getRoll()));
        opMode.telemetry.addData("imu pitch: ", (imu.getRobotYawPitchRollAngles().getPitch()));
        opMode.telemetry.addData("imu yaw: ", (imu.getRobotYawPitchRollAngles().getYaw()));

        opMode.telemetry.addData("Slide Arm Rotator Left Touched: ", !touchSensorSlideArmRotatorLeft.isPressed());
        opMode.telemetry.addData("Slide Arm Rotator Right Touched: ", !touchSensorSlideArmRotatorRight.isPressed());
        opMode.telemetry.addData("Slide Arm Rotator in RUN_TO_POSITION? ", slideArmRotatorRunningToPosition);
        //opMode.telemetry.addData(">", "rotateArms - use triggers for control" );

        opMode.telemetry.addData("Green Intake BOX ROTATOR Pos: ", intakeBoxRotatorPosition);
        //opMode.telemetry.addData(">", "green intake box rotator - use dpad L/R for control" );
        // color sensor data PLEASE do not delete!
        /* COLOR SENSOR DISCONNECTED 2/4/2025
        opMode.telemetry.addData("colorDetected: " , ColorRevV3Sensor().toString());
        opMode.telemetry.addData("Blinkin Left: ", Blinken_pattern.toString());
        opMode.telemetry.addData("Hue: " , hsvValues[0]);
        opMode.telemetry.addData("Sat: " , hsvValues[1]);
        opMode.telemetry.addData("Val: " , hsvValues[2]);
        opMode.telemetry.addData("Swiper Position:", swiper_position);
        showColorSensorTelemetry();
         */

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

    private int targetPositionIMUARM_ticks = 500;
    private double targetIMU_Degrees = 0.0;
    public int findRealArmAngle(double targetDegrees) {
        targetIMU_Degrees = targetDegrees;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        imuPosition = (orientation.getRoll());

        targetPositionIMUARM_ticks = (int) ((targetIMU_Degrees - imuPosition) *  ticks_in_degrees);

        opMode.telemetry.addData("imu position: " , imuPosition);
        return targetPositionIMUARM_ticks;
    }
}
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@TeleOp(name="Test FourArmMotors_LimitSwitches", group="Test")
//@Disabled
public class TestFourArmMotors_LimitSwitches extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftRotater;
    DcMotor rightRotater;
    DcMotor slideLeft;
    DcMotor slideRight;
    TouchSensor touchSensorRight;
    TouchSensor touchSensorLeft;
    TouchSensor touchSensorRotator;
    int rotatorSetPosition = 0;

    //  private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
  /*  static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

   */

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        // ************* Slide MOTORS ****************
        leftRotater = hardwareMap.get(DcMotorEx.class, "leftRotater");
        rightRotater = hardwareMap.get(DcMotorEx.class, "rightRotater");
        slideLeft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slidesRight");

        touchSensorRight = hardwareMap.get(TouchSensor.class, "sensor_touchRight");
        touchSensorLeft = hardwareMap.get(TouchSensor.class, "sensor_touchLeft");
        touchSensorRotator = hardwareMap.get(TouchSensor.class, "sensor_touchRotate");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRotater.setDirection(DcMotor.Direction.REVERSE);
        rightRotater.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        leftRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //idk if this is necessary...
        rightRotater.setTargetPosition(rotatorSetPosition);
        leftRotater.setTargetPosition(rotatorSetPosition);
        leftRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting rotators at", "%7d :%7d",
                leftRotater.getCurrentPosition(),
                rightRotater.getCurrentPosition());
        telemetry.addData("Starting sliders at", "%7d :%7d",
                slideLeft.getCurrentPosition(),
                slideRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        leftRotater.setPower(1.0);
        rightRotater.setPower(1.0);
        while (opModeIsActive()) {
            //todo 45=-250 60=-450 90=-820 extreme hang -1200
            // Horizontal extension motors
            if (gamepad2.left_stick_y > 0.1) {
                slideLeft.setPower(gamepad2.left_stick_y);
                slideRight.setPower(gamepad2.left_stick_y);
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad2.left_stick_y <= 0.1) {
                slideLeft.setPower(gamepad2.left_stick_y);
                slideRight.setPower(gamepad2.left_stick_y);
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }

            else
            {
                  slideLeft.setPower(0);
                  slideRight.setPower(0);
            }


            //Rotater Motors
            if (gamepad2.right_stick_y> 0.1) {
               rotatorSetPosition += Math.round(gamepad2.right_stick_y * 10);

            } else if (gamepad2.right_stick_y <= -0.1) {
                rotatorSetPosition += Math.round(gamepad2.right_stick_y * 10);

            }
            else if (gamepad2.circle) {
                rotatorSetPosition = 250;
            }
            else if (gamepad2.triangle) {
                rotatorSetPosition = 450;
            }
            else if (gamepad2.square) {
                rotatorSetPosition = 820;
            }
            else if (gamepad2.cross) {
                rotatorSetPosition = 0;
            }
            else
            {
                //rightRotater.setPower(0);
               // leftRotater.setPower(0);
            }
            if (rotatorSetPosition < 0) {
                rotatorSetPosition = 0;
            }
            rightRotater.setTargetPosition(-rotatorSetPosition);
            leftRotater.setTargetPosition(-rotatorSetPosition);

            /*
            // TOUCH SENSORS
            if (touchSensorRight.isPressed()) {
                telemetry.addData("Touch Sensor Right", "Is Pressed");
                slideRight.setPower(0);
                slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               // theRobot.motorpower = 0.0;
            } else {
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Touch Sensor Right", "Is Not Pressed");
            }
            if (touchSensorLeft.isPressed()) {
                telemetry.addData("Touch Sensor Left", "Is Pressed");
                slideLeft.setPower(0);
                slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Touch Sensor Left", "Is Not Pressed");
            }

            //Rotaters Touch Sensor. Only Use ONE because of the connection.
            if (touchSensorRotator.isPressed()) {
                telemetry.addData("Touch Sensor Rotate", "Is Pressed");
                leftRotater.setPower(0);
                rightRotater.setPower(0);
                leftRotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightRotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            } else {
                leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Touch Sensor Rotater", "Is Not Pressed");
            }
*/


            telemetry.addData("Motor Ticks slideLeft: ", slideLeft.getCurrentPosition());
            telemetry.addData("Motor Ticks slideRight: ", slideRight.getCurrentPosition());
            telemetry.addData("Motor Ticks rightRotater: ", rightRotater.getCurrentPosition());
            telemetry.addData("Motor Ticks leftRotater: ", leftRotater.getCurrentPosition());
            telemetry.addData("Target Rotator Position ", rotatorSetPosition);


            telemetry.update();
            sleep(100);

          //  telemetry.addData("Motor Ticks slideLeft: ", slideLeft.getCurrentPosition());
          //  telemetry.addData("Motor Ticks slideRight: ", slideRight.getCurrentPosition());


            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
    /*    encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

     */

            /*
             *  Method to perform a relative move, based on encoder counts.
             *  Encoders are not reset as the move is based on the current position.
             *  Move will stop if any of three conditions occur:
             *  1) Move gets to the desired position
             *  2) Move runs out of time
             *  3) Driver stops the OpMode running.
             */
        /*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

         */
        }
    }
}

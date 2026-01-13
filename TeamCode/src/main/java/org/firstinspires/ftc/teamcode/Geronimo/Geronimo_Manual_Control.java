package org.firstinspires.ftc.teamcode.Geronimo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Geronimo 1 Manual Control")
@Disabled
public class Geronimo_Manual_Control extends LinearOpMode {
    Geronimo theRobot;
    boolean rotatorPowerApplied = false;
    boolean slidePowerApplied = false;

    boolean alignBusy = false;

    private boolean dpadDownPrev = false;
    //boolean intakeStarPowerApplied = false;



    // Scrimmage Meet Ideas:
    // Press intake button, have claw grab the sample
    // Have claw go down when slides extend forward?
    // 1 button to grab specimen off wall (rotate slides, Limelight correction, grab, rotate claw up to get off wall)

    public void runOpMode() {
        theRobot = new Geronimo(true, true, this);
        telemetry.setMsTransmissionInterval(11);

        theRobot.Init(this.hardwareMap);
        theRobot.InitLimelight(hardwareMap);
        theRobot.ShowTelemetry();
        telemetry.update();
        theRobot.InitRoadRunner(hardwareMap);

        waitForStart();
        resetRuntime();
        theRobot.HooksReleased();
        theRobot.SetSwiperPosition(Geronimo.SWIPER_MAX_POS);
        theRobot.SetSwiper2Position(Geronimo.SWIPER2_MIN_POS);
        theRobot.SetPIDF_Enabled(true);
        theRobot.SetPIDF_Slides_Enabled(true);

        // create a thread to control the rotator arm position with PIDF control
        Thread pidfThread = new Thread(() -> {
            while (!isStopRequested()) {
             theRobot.SetSlideRotatorArmToPositionPIDF();
             theRobot.SetSlideExtensionToPositionPIDF();

             sleep(20);
            }
        });

         pidfThread.start();

        while (opModeIsActive()) {
            theRobot.EndgameBuzzer();
            /* *************************************************
             *************************************************
             * Driver Controls (gamepad1)
             *************************************************
             *************************************************

             */
            // Drive Controls uses left_stick_y, left_stick_x, and right_stick_x
            theRobot.RunDriveControls();
            theRobot.UpdateLimelightStatusAndResults();

            // boolean dpadDownNow = gamepad1.dpad_down && gamepad1.options;

            if (gamepad1.dpad_right && !gamepad1.options) {
                theRobot.ReadyHangRobot();
            }

            if (gamepad1.right_bumper && !alignBusy) {
                alignBusy = true;

                // remember original slide‐PIDF setting
                boolean pidfSlidesOrig = Geronimo.pidfSlidesEnabled;

                // so then we will disable slide PIDF so RoadRunner isn’t fighting the  strafing
                theRobot.SetPIDF_Slides_Enabled(false);


                theRobot.AlignOnFloorSampleWithPercent();

                // now we will set the slide PIDF to whatever it was before
                theRobot.SetPIDF_Slides_Enabled(pidfSlidesOrig);

                alignBusy = false;
            }

            // 3) then all your other gamepad1 / gamepad2 commands
            if (gamepad1.triangle && !gamepad1.options) {
                theRobot.imu.resetYaw();
            } else if (gamepad1.triangle && gamepad1.options) {
                theRobot.SetFieldCentricMode(true);

            } else if (gamepad1.square && gamepad1.options) {
                theRobot.SetFieldCentricMode(false);
            }

            // RESERVED COMBOS    options + cross and options + circle

            // Press the triangle button / "y" while facing directly away from the driver to set the IMU correctly for field-centric mode if off
            if (gamepad1.triangle && !gamepad1.options) {
                theRobot.imu.resetYaw();
            }
            // field centric drive mode (default at startup)
            else if (gamepad1.triangle && gamepad1.options)
            {
                theRobot.SetFieldCentricMode(true);
            }
            // robot centric drive mode
            else if (gamepad1.square && gamepad1.options)
            {
                theRobot.SetFieldCentricMode(false);
            }

            // dpad left WITHOUT options is for the swipers
            else if (gamepad1.dpad_left && !gamepad1.options) {
                theRobot.SetSwiperPosition(Geronimo.SWIPER_MAX_POS);
                theRobot.SetSwiper2Position(Geronimo.SWIPER2_MIN_POS);
                theRobot.SpecialSleep(500);
                theRobot.SetSwiperPosition(Geronimo.SWIPER_MIN_POS);
                theRobot.SetSwiper2Position(Geronimo.SWIPER2_MAX_POS);
                theRobot.SpecialSleep(500);
                theRobot.SetSwiperPosition(Geronimo.SWIPER_MAX_POS);
                theRobot.SetSwiper2Position(Geronimo.SWIPER2_MIN_POS);
            }
            // LEVEL-3 Hang Logic
            else if(gamepad1.dpad_up && !gamepad1.options){
               theRobot.level3Ascent();
            }
            // LEVEL-2 Hang Logic
            //else if(gamepad1.left_bumper) {
            //    theRobot.level2Ascent();
            //}
            // Emergency pause
            else if (gamepad1.share) {
              // theRobot.SetSlidesToHoldCurrentPosition();
                // theRobot.SetSlideRotatorArmToHoldCurrentPosition();
                // theRobot.ReadyHangRobot();
            }
            // LOCKS
            /*
            else if (gamepad1.dpad_right && !gamepad1.options) {
                theRobot.HooksReleased();
            }
            else if(gamepad1.dpad_down && !gamepad1.options){
                theRobot.HooksLocked();
            }
            */

            // LIMELIGHT Color Target

            //else if(!gamepad1.squapre && !gamepad1.options && alignBusy){
              //  alignBusy = false
            //}

            else if (gamepad1.cross && !gamepad1.options) {
                theRobot.SetLimelightToRed();

            }
            else if (gamepad1.circle && !gamepad1.options && gamepad2.left_stick_y < 0.1 && gamepad2.left_stick_y > -0.1) {
                theRobot.SetLimelightToBlue();
            }

            // AVAILABLE COMMANDS
            // TEMP -- turn limelight on/off
            else if(gamepad1.dpad_right && gamepad1.options){
                theRobot.SetLimelightEnabled(!Geronimo.limelightEnabled);
            }
            // TEMP -- Switch Limelight to different pipelines/models
            else if(gamepad1.dpad_down && !gamepad1.options){
                theRobot.SwitchLimelightPipeline();
            }
            // dpadDownPrev = dpadDownNow;

            /* *************************************************
             *************************************************
             * Arm Controls (gamepad2)
             *
             *************************************************
             *************************************************
             */
            // SLIDE MOTOR CONTROL through the LEFT STICK Y (up is negative)

            if ((gamepad2.left_stick_y > 0.1 && !gamepad2.options) || (gamepad2.left_stick_y <= -0.1 && !gamepad2.options)) {
                if(!theRobot.pidfSlidesEnabled){
                    slidePowerApplied = true;
                    // If commanding an override to go down
                    if (gamepad2.left_stick_y > 0.1 && gamepad1.circle && !gamepad1.options){
                        theRobot.SetSlidesToPowerMode(-gamepad2.left_stick_y);
                    }
                    // else if slide limit pressed and still commanding down
                    else if (theRobot.GetSlidesLimitSwitchPressed() && (gamepad2.left_stick_y > 0.1)) {
                        theRobot.ResetSlidesToZero();
                    } else {
                        theRobot.SetSlidesToPowerMode(-gamepad2.left_stick_y);
                    }

                } else {
                    int slideScale = 1000;
                    theRobot.SetSlideToPosition(theRobot.GetSlideLeftCurrentPosition() - (int)(slideScale * gamepad2.left_stick_y));


                }

            }
            // else if was moving the slides through the LEFT STICK Y and stopped -- tell the slides to hold the current position

            else if (slidePowerApplied && !theRobot.pidfSlidesEnabled) {
                slidePowerApplied = false;
                theRobot.SetSlidesToPowerMode(0.0);
                if (theRobot.GetSlidesLimitSwitchPressed()) {
                    theRobot.ResetSlidesToZero();
                }
            }
            // else if the slide was running to a set position
            //TODO -- need to better integrate with PIDF
           else if (theRobot.GetSlidesRunningToPosition() && !theRobot.pidfSlidesEnabled)
            {
                // if slides were running to zero and the limit switch got pressed
                if (theRobot.GetSlidesTargetPosition() == 0 && theRobot.GetSlidesLimitSwitchPressed()) {
                    theRobot.ResetSlidesToZero();
                }
                // else slides just running somewhere else, check if should just hold current position
                else {
                    int leftDifference = Math.abs(theRobot.GetSlidesTargetPosition() - theRobot.GetSlideLeftCurrentPosition());
                    int rightDifference = Math.abs(theRobot.GetSlidesTargetPosition() - theRobot.GetSlideRightCurrentPosition());
                    if ((leftDifference <= 2) && (rightDifference <= 2)) {
                        theRobot.SetSlidesToHoldCurrentPosition();
                    }
                }
            }


            // Make sure the slides aren't ever trying to go past their horizontal limits
            //TODO -- need to better integrate with PIDF
            if(!theRobot.pidfSlidesEnabled){
                theRobot.Slides_Horizontal_MAX_Limit();
            }



            // small hanger arms holding the urchin / green box
            if (gamepad2.right_stick_x > 0.1 && gamepad2.options) {
                theRobot.SetSmallArmHangerIncrementUp();
            } else if (gamepad2.right_stick_x  < -0.1 && gamepad2.options) {
                theRobot.SetSmallArmHangerDecrementDown();
            }

            // Green Box Control / Urchin angle
            if (gamepad2.left_stick_x > 0.1 && gamepad2.options) {
                theRobot.SetIntakeBoxRotatorDecrementDown();
            } else if (gamepad2.left_stick_x < -0.1 && gamepad2.options) {
                theRobot.SetIntakeBoxRotatorIncrementUp();
            }

            // Slide Rotator Controls
            if (gamepad2.right_stick_y < -0.25 && !Geronimo.pidfEnabled) {
                rotatorPowerApplied = true;
                theRobot.SetSlideRotatorToPowerMode(0.4);
            }
            else if (gamepad2.right_stick_y > 0.25 && !Geronimo.pidfEnabled) {
                rotatorPowerApplied = true;
                theRobot.SetSlideRotatorToPowerMode(-0.4);
            }
            // if was manually powering rotator and now let go, tell the arms to hold at this position
            else if (rotatorPowerApplied && !theRobot.GetRotatorArmRunningToPosition() && !theRobot.GetSlideRotatorArmLimitSwitchPressed()) {
                rotatorPowerApplied = false;
                theRobot.SetSlideRotatorArmToPosition(theRobot.GetRotatorLeftArmCurrentPosition());
            }
            else if (theRobot.GetRotatorArmRunningToPosition()) {
                if (theRobot.GetRotatorArmTargetPosition() == 0 && theRobot.GetSlideRotatorArmLimitSwitchPressed()) {
                    theRobot.ResetSlideRotatorArmToZero();

                } else {
                    int leftDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorLeftArmCurrentPosition());
                    int rightDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorRightArmCurrentPosition());
                    if ((leftDifference <= 2) && (rightDifference <= 2)) {
                        theRobot.SetSlideRotatorArmToHoldCurrentPosition();
                    }
                }
            }


            // Urchin Open/Close Control
            if (gamepad2.right_trigger > 0.2 && !gamepad2.options) {
                theRobot.SetUrchinServoPosition(0);
            }
            else if (gamepad2.left_trigger > 0.2 && !gamepad2.options) {
                theRobot.SetUrchinServoPosition(1);
            }

            /*
            // Urchin Open/Close Control
            if (gamepad2.right_bumper && !gamepad2.options) {
                double waitTime = getRuntime() + 0.25;
                if (theRobot.urchinServo_position == 1 && getRuntime() < waitTime) {
                    theRobot.SetUrchinServoPosition(0);
                }
                else if (theRobot.urchinServo_position == 0 && getRuntime() < waitTime) {
                    theRobot.SetUrchinServoPosition(1);
                }
            }
            */


            // Combo Moves for specimen deliveries
            if (gamepad2.square && !gamepad2.options) {
                theRobot.SpecimenPickupFromWall();
            } else if (gamepad2.cross && !gamepad2.options) {
                theRobot.RemoveFromWall();
            } else if (gamepad2.circle && !gamepad2.options) {
                theRobot.SpecimenDeliverHighChamberAlternate();
            } else if (gamepad2.triangle && !gamepad2.options) {
                theRobot.SpecimenDeliverHighChamberFinishingMove();
            }

            //Combo moves for urchin specimen deliveries
            //else if (gamepad2.left_bumper && gamepad2.options) {
             //   theRobot.UrchinPickupFromWall();
            //} else if (gamepad2.right_bumper && gamepad2.options){
            //    theRobot.UrchinRemoveFromWall();
            //} else if (gamepad2.left_trigger > 0.2 && gamepad2.options) {
            //    theRobot.UrchinDeliverHighChamberAlternate();
            //} else if (gamepad2.right_trigger > 0.2 && gamepad2.options) {
            //    theRobot.UrchinDeliverHighChamberFinishingMove();
            //}

            //TEMP? -- PIDF Enabled toggle
            else if(gamepad2.dpad_up && gamepad2.options){
                theRobot.SetPIDF_Enabled(!Geronimo.pidfEnabled);
            }
            
            // Combo moves for basket deliveries
            else if(gamepad2.dpad_left && !gamepad2.options) {

                theRobot.SampleUrchinFloorPickup();
            }
            else if (gamepad2.dpad_left && gamepad2.options) {
                theRobot.SampleUrchinFloorJam();
            } else if (gamepad2.dpad_down && !gamepad2.options){
                theRobot.SampleUrchinFloorPickupFinishingMove();
            } else if (gamepad2.share) {
                theRobot.Stow();
            } else if (gamepad2.dpad_right && !gamepad2.options) {
                theRobot.BasketHigh();
            } else if (gamepad2.dpad_up && !gamepad2.options) {
                theRobot.BasketHighFinishingMove();
            }

            // Inspection controls
            else if (gamepad2.square && gamepad2.options) {
                theRobot.InspectionLowForward();
            }
            else if (gamepad2.triangle && gamepad2.options) {
                theRobot.InspectionHighPos();
            }


            // Claw Control
            if (gamepad2.left_bumper && !gamepad2.options) {
                theRobot.SetClawPosition(Geronimo.CLAW_MAX_POS);
            } else if (gamepad2.right_bumper && !gamepad2.options) {
                theRobot.SetClawPosition(Geronimo.CLAW_MIN_POS);
            }

            /*
            // Claw Control
            if (gamepad2.left_bumper && !gamepad2.options) {
                double waitTime = getRuntime() + 0.25;
                if (theRobot.claw_position == Geronimo.CLAW_MIN_POS && getRuntime() < waitTime) {
                    theRobot.SetClawPosition(Geronimo.CLAW_MAX_POS);
                }
                else if (theRobot.claw_position == Geronimo.CLAW_MAX_POS && getRuntime() < waitTime) {
                    theRobot.SetClawPosition(Geronimo.CLAW_MIN_POS);
                }
            }
            */

            theRobot.ShowTelemetry();
        } // end while (opModeIsActive())

        // end the arm control thread
        pidfThread.interrupt();

    }
}

package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.FtcWifiDirectChannelSelectorActivity;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@Config
@TeleOp
public class StarterBot extends LinearOpMode{
    //hook offset currently off - five inches from sub
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    SharkDrive shark = new SharkDrive();
    MoveCommand moveCmd = new MoveCommand();
    ElapsedTime runtime = new ElapsedTime();
//    opencv cv = new opencv();
    Extension extend = new Extension();

    boolean canShift = true;
    double currentSpeed = 1;
    double speedInterval = 0.4;

    double diffSpeed = 1;

    double localOffset = -4;
    double localOffsetIncrement = 3;
    double grabDistance = 0; // 11
    double hookDistance = 27; // 43

    boolean canShiftArm = true;

    boolean normalControl = true;


    boolean canStartSubMacro = true;
    boolean canStartGrabMacro = true;
    boolean canStartHookMacro = true;
    double localWristPos = 0;

    double preciseLenience = 0.6;
    double arcLenience = 10;

    double xLocalRehomePos = 0;

    double yLocalRehomePos = 0;

    double xRehomePos = 0; // 2
    double yRehomePos = 0; // 31.5

    boolean canGrab = false;
    boolean canSetDiffPos = true;

    double extensionRollSpeed = 1;

    int hookHeight = 1100;
    int grabHeight = 50;
    int extensionSafetyThreshold = -200;

    // inches
    double extendLength = 19.5;

    int extendPos = 0;

    double clawLength = 3.5; // inches

    @Override
    public void runOpMode()
    {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        dt.init(hardwareMap);
        cs.init(hardwareMap, false);
        am.init(hardwareMap);
        shark.init(hardwareMap, true);
        moveCmd.init(hardwareMap, false);
//        cv.init(hardwareMap);
        extend.init(hardwareMap);

        waitForStart();
        am.SetArmSpeed(0.45);
        cs.SetDiffPos(0);
        am.Rotate(0,'T');
        am.SetLocalNeutral(50);
        am.Rotate(0,'T');

//        cv.StartStream(telemetry);
        while(opModeIsActive())
        {
            // triggers should be extension
            // joystick should be wrist movement
            // swap between normal and custom in teleop for wrist
            double targetPowerX = gamepad1.left_stick_x;
            double targetPowerY = -gamepad1.left_stick_y;
            double targetRotation = gamepad1.right_stick_x;

            boolean aButtonPressed = gamepad1.a;
            boolean xButtonPressed = gamepad1.x;
            boolean yButtonPressed = gamepad1.y;
            boolean bButtonPressed = gamepad1.b;

            boolean dpadUpPressed = gamepad1.dpad_up;
            boolean dpadDownPressed = gamepad1.dpad_down;

            boolean leftBumperPressed = gamepad1.left_bumper;
            boolean rightBumperPressed = gamepad1.right_bumper;

            double leftTriggerPressed = gamepad1.left_trigger;
            double rightTriggerPressed = gamepad1.right_trigger;

            //arm - add encoders later
            double wristInputX = gamepad2.right_stick_x; // usually left
            double wristInputY = gamepad2.right_stick_y;

            double armInput = gamepad2.left_stick_y;

            double clawOpen = gamepad2.left_trigger;
            double clawClose = gamepad2.right_trigger;

            boolean dpadUpPressed2 = gamepad2.dpad_up;
            boolean dpadDownPressed2 = gamepad2.dpad_down;
            boolean dpadLeftPressed2 = gamepad2.dpad_left;
            boolean dpadRightPressed2 = gamepad2.dpad_right;

            boolean armToggle = gamepad2.left_bumper;

            boolean aButtonPressed2 = gamepad2.a;
            boolean bButtonPressed2 = gamepad2.b;
            boolean xButtonPressed2 = gamepad2.x;
            boolean yButtonPressed2 = gamepad2.y;

            TelemetryPrint();

            if (normalControl && canStartSubMacro)
            {
                targetPowerX *= currentSpeed;
                targetPowerY *= currentSpeed;
                targetRotation *= currentSpeed;

                dt.FieldOrientedTranslate(targetPowerX,targetPowerY,targetRotation, shark.GetOrientation());
                ResetLocalOffset(dpadUpPressed);
                SpeedShift(leftBumperPressed,rightBumperPressed);
//
//                Rehoming(dpadDownPressed);
//
//                ResetTopEncoder(yButtonPressed2);
//                ResetDownEncoder(xButtonPressed2);
                ArmSpeedToggle(armToggle);
                ClawControl(clawClose,clawOpen);
                ExtensionClawControl(clawClose, clawOpen);

                ExtensionControl(dpadUpPressed2,dpadDownPressed2);
                ExtensionManipulatorControl(dpadRightPressed2,dpadLeftPressed2,aButtonPressed2,bButtonPressed2);

                if (!canGrab)
                {
                    ArmControl(-armInput);
                }
                DiffControl(wristInputX);

                extend.Update();
            }

            runtime.reset();
//            SubmersibleGrab(aButtonPressed);
            HookMacro(yButtonPressed);
            GrabMacro(bButtonPressed);
            cs.Update();
        }
//        cv.StopStream();
    }

    private void Rehoming(boolean pressed)
    {
        if (!pressed) {return;}

        shark.Rehome();
        xLocalRehomePos = xRehomePos;
        yLocalRehomePos = yRehomePos;
    }

    private void ResetTopEncoder(boolean pressed)
    {
        if (pressed)
        {
            am.ResetEncodersUp();
        }
    }

    private void ResetDownEncoder(boolean pressed)
    {
        if (pressed)
        {
            am.ResetEncoders();
        }
    }

    private void ResetLocalOffset(boolean dpadUpPressed)
    {
        if (dpadUpPressed)
        {
            localOffset = 3;
        }
    }

    private void SpeedShift(boolean left, boolean right)
    {
        if (left && canShift)
        {
            if (currentSpeed - speedInterval > 0)
            {
                currentSpeed -= speedInterval;
                canShift = false;
            }
        }
        else if (right && canShift)
        {
            if (currentSpeed + speedInterval <= 1)
            {
                currentSpeed += speedInterval;
                canShift = false;
            }
        }
        else if (!left && !right)
        {
            canShift = true;
        }
    }

//    private void SubmersibleGrab(boolean a)
//    {
//        if (a && canStartSubMacro)
//        {
//            normalControl = false;
//            canStartSubMacro = false;
//            cv.SetDetecting(true);
//            boolean shouldReturn = false;
//
//            shouldReturn = TeleopMoveCommandCV(0.8, 0, 0, 0,preciseLenience,2,1,grabHeight, false, false, 'm');
//            cv.SetDetecting(false);
//            if (shouldReturn) {return;}
//
//            shouldReturn = TeleopMoveCommandCV(0, 0, 0, 0,preciseLenience,2,1,grabHeight, false, false, 'e');
//            if (shouldReturn) {return;}
//
//            shouldReturn = TeleopMoveCommandCV(0, 0, 0, 0,preciseLenience,2,1,grabHeight, false, false, 'r');
//            if (shouldReturn) {return;}
//            sleep(400);
//
//            shouldReturn = TeleopMoveCommandCV(0, 0, 0, 0,preciseLenience,2,1,grabHeight, false, false, 'p');
//            if (shouldReturn) {return;}
//            sleep(400);
//
//            shouldReturn = TeleopMoveCommandCV(0, 0, 0, 0,preciseLenience,2,1,grabHeight, false, true, 'c');
//            if (shouldReturn) {return;}
//            sleep(400);
//
//            shouldReturn = TeleopMoveCommandCV(0, 0, 0, 0,preciseLenience,2,1,grabHeight, false, true, 'i');
//            if (shouldReturn) {return;}
//
//
//            am.SetLocalNeutral(am.GetTargetPosition());
//            normalControl = true;
//        }
//        else if (!a)
//        {
//            am.SetLocalNeutral(am.GetTargetPosition());
//            canStartSubMacro = true;
//        }
//    }

    private void HookMacro(boolean y)
    {
        if (y && canStartHookMacro)
        {
            normalControl = false;
            canStartHookMacro = false;
            TeleopMoveCommandY(1,localOffset, grabDistance+8, 0-shark.GetLastValidIMUReading(),preciseLenience,0,0.7,grabHeight,0, 0,0,true, 'C', false);
//            TeleopMoveCommandY(1,localOffset, hookDistance-5, 0-shark.GetLastValidIMUReading(),preciseLenience,1,0.7,grabHeight,0, 0,0,true, 'C', false);

            TeleopMoveCommandY(0.5,localOffset, hookDistance, 0-shark.GetLastValidIMUReading(),preciseLenience,1,1,grabHeight,0, 0,0,true, 'C', false);
            TeleopMoveCommandY(0,localOffset, hookDistance, 0-shark.GetLastValidIMUReading(),arcLenience,2,1,hookHeight,0, 0,0,true, 'C', false);
            TeleopMoveCommandY(0,localOffset, hookDistance, 0-shark.GetLastValidIMUReading(),arcLenience,2,1,hookHeight,0, 0,0,false, 'C', false);
            TeleopMoveCommandY(1,localOffset,hookDistance-8,0-shark.GetLastValidIMUReading(),preciseLenience,1,1,hookHeight,0,0,0,false,'C',false);

            localOffset += localOffsetIncrement;
            am.SetLocalNeutral(am.GetTargetPosition());
            normalControl = true;
        }
        else if (!y)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartHookMacro = true;
        }
    }

    private void GrabMacro(boolean b)
    {
        if (b && canStartGrabMacro)
        {
            normalControl = false;
            canStartGrabMacro = false;
            boolean shouldReturn = false;
            shouldReturn = TeleopMoveCommandB(1, 40, grabDistance+8, 0-shark.GetLastValidIMUReading(),preciseLenience,2,1, grabHeight, 0,0,0, false, 'G', false);
            if (shouldReturn) {return;}

            SparkFunOTOS.Pose2D overridePos = new SparkFunOTOS.Pose2D(40,grabDistance+8,shark.GetImuReading());
            shark.OverrideOtosPos(overridePos);

            shouldReturn = TeleopMoveCommandB(0.5, 40, grabDistance, 0-shark.GetLastValidIMUReading(),preciseLenience,1,1, grabHeight, 0,0,0, false, 'G', false);
            if (shouldReturn) {return;}

            shouldReturn = TeleopMoveCommandB(0, 40, grabDistance, 0-shark.GetLastValidIMUReading(),arcLenience,2,1, grabHeight, 0,0,0, true, 'G', false);
            if (shouldReturn) {return;}
            sleep(400);

            shouldReturn = TeleopMoveCommandB(0, 40, grabDistance, 0-shark.GetLastValidIMUReading(), arcLenience,2,1, grabHeight + 200, 0,0,0, true, 'G', false);
            if (shouldReturn) {return;}

            am.SetLocalNeutral(am.GetTargetPosition());
            normalControl = true;
        }
        else if (!b)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartGrabMacro = true;
        }
    }

//    private boolean TeleopMoveCommandCV(double speed, double x, double y, double h, double d, int axis, double speedA, int a, boolean claw, boolean extendClaw, char type)
//    {
//        do
//        {
////            TelemetryPrint();
//            cs.SetClawOpen(claw);
//            TelemetryPrint();
//
//            // set extension to max
//            int e = -1500;
//
//            // fix < 0 problem
//            double offsetDist = cv.GetDistance() - clawLength;
//
//            if (offsetDist < 0)
//            {
//                offsetDist = 0;
//            }
//
//            //  convert extension into 0-1 scale
//            double multiplier = offsetDist / extendLength;
//
//            if (multiplier > 1) { multiplier = 1; }
//
//            e *= multiplier;
//
//            extendPos = e;
//
//            // checks if the object is within reach, if it is do not move
//            double correctDist = cv.GetDistance() - extendLength;
//            if (correctDist < -3)
//            {
//                correctDist = 0;
//            }
//
//            double theta = cv.GetTheta();
//
//            double rollWrist = 0;
//
//            if (theta > 90)
//            {
//                double normalizedTheta = (theta - 90)/90;
//                rollWrist = normalizedTheta * 0.35;
//            }
//            else
//            {
//                double normalizedTheta = theta/90;
//                rollWrist = (normalizedTheta * 0.35) + 0.35;
//            }
//
//            double pitchWrist = 0.33; // 0.15 offset
//
//            double correctXOffset = cv.GetXOffset();
//
//            if (type == 'i')
//            {
//                moveCmd.MoveToPositionCV(0,x,y,h,100,axis,speedA,a,0,claw,0,0, correctXOffset, 0, true, 'e', cv.GetFirstDetection());
//                extend.CloseExtendClaw();
//                extend.UpdateOverride();
//                extend.Update();
//            }
//            else
//            {
//                moveCmd.MoveToPositionCV(speed,x,y,h,d,axis,speedA,a,e,claw,rollWrist,pitchWrist, correctXOffset, correctDist, extendClaw, type, cv.GetFirstDetection());
//            }
//
//            if (!gamepad1.a) {
//                normalControl = true;
//                cv.SetDetecting(false);
//                am.SetLocalNeutral(a);
//                cv.SetFirstDetection(true);
//                return true;
//            }
//        } while (!moveCmd.GetCommandState());
//        cv.SetFirstDetection(true);
//        return false;
//    }

    private void TeleopMoveCommandA(double speed, double x, double y, double h, double d, int axis, double speedA, int a, int e, double roll, double pitch, boolean claw, char wrist, boolean extendClaw)
    {
        do
        {
//            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed,x,y,h,d,axis,speedA,a,e,roll, pitch, claw,wrist,extendClaw);
            if (!gamepad1.a) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

    private boolean TeleopMoveCommandB(double speed, double x, double y, double h, double d, int axis, double speedA, int a, int e, double roll, double pitch, boolean claw, char wrist, boolean extendClaw)
    {
        shark.OdometryControl(0, x, y, h, d, axis);
        moveCmd.InitializeTeleopCommand();
        do
        {
            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed * currentSpeed,x,y,h,d,axis,speedA,a,e,roll, pitch, claw,wrist,extendClaw);
            if (!gamepad1.b) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return true;
            }
        } while (!moveCmd.GetCommandState());
//        shark.DeactivateBoolsCompleted();
        return false;
    }

    private void TeleopMoveCommandY(double speed, double x, double y, double h, double d, int axis, double speedA, int a, int e, double roll, double pitch, boolean claw, char wrist, boolean extendClaw)
    {
        shark.OdometryControl(0, x, y, h, d, axis);
        moveCmd.InitializeTeleopCommand();
        do
        {
            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed * currentSpeed,x,y,h,d,axis,speedA,a,e,roll, pitch, claw,wrist,extendClaw);
            if (!gamepad1.y) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
//        shark.DeactivateBoolsCompleted();
    }

//    private void SharkDriveRelocalize()
//    {
//        shark.SetLastLimelightPosition(lime.GetLastPosition());
//        shark.SetLastValidIMUReading();
//        shark.Rehome();
//    }

//    private void UpdateLimelight()
//    {
//        lime.GetLimelightData(false, shark.GetOrientation());
//    }

    private void ArmSpeedToggle(boolean armToggle)
    {
        if (armToggle)
        {
            if (!canShiftArm) { return; }
            if (am.GetArmSpeed() == ArmLiftMotor.ArmSpeeds.FAST.speed)
            {
                am.SetArmSpeed(ArmLiftMotor.ArmSpeeds.SLOW.speed);
                canShiftArm = false;
            }
            else if (am.GetArmSpeed() == ArmLiftMotor.ArmSpeeds.SLOW.speed)
            {
                am.SetArmSpeed(ArmLiftMotor.ArmSpeeds.FAST.speed);
                canShiftArm = false;
            }
        }
        else
        {
            canShiftArm = true;
        }
    }

    private void ClawControl(double open, double close)
    {
        if (open > 0.5)
        {
            cs.SetClawOpen(true);
        }
        else if (close > 0.5)
        {
            cs.SetClawOpen(false);
        }
    }

    private void ArmControl(double input)
    {
        if (input > 0.2)
        {
            am.Rotate(1,'T');
        }
        else if (input < -0.2)
        {
            am.Rotate(-1,'T');
        }
        else
        {
            am.Rotate(0,'T');
        }
    }

    private void ExtensionControl(boolean positive, boolean negative)
    {
        if (positive)
        {
            extend.MoveExtend(1,'T');
        }
        else if (negative)
        {
            if (extend.GetPitchLocalPosition() == 0.33 && extend.GetExtensionPosition() > extensionSafetyThreshold)
            {
                extend.MoveExtend(0, 'T');
            }
            else
            {
                extend.MoveExtend(-1,'T');
            }
        }
        else
        {
            extend.MoveExtend(0,'T');
        }
    }

    private void ExtensionManipulatorControl(boolean positiveRoll,boolean negativeRoll,boolean downPitch,boolean upPitch)
    {
        double roll = extend.GetRollLocalPosition();
        double pitch = extend.GetPitchLocalPosition();

        // roll
        if (positiveRoll)
        {
            roll += extensionRollSpeed * runtime.seconds();
        }
        else if (negativeRoll)
        {
            roll -= extensionRollSpeed * runtime.seconds();
        }

        roll = Range.clip(roll, 0, 1);

        // pitch
        if (downPitch)
        {
            if (extend.GetExtensionPosition() < extensionSafetyThreshold)
            {
                pitch = 0.33;
            }
        }
        else if (upPitch)
        {
            pitch = 0;
        }

        extend.SetLocalManipulatorState(roll, pitch);
    }

    private void ExtensionClawControl(double left, double right)
    {
        if (left > 0.5)
        {
            extend.CloseExtendClaw();
        }
        else if (right > 0.5)
        {
            extend.OpenExtendClaw();
        }
    }

    private void DiffControl(double wristInputX)
    {
        if (wristInputX > 0.1)
        {
            UpdateDiffPos(diffSpeed*runtime.seconds());
        }
        else if (wristInputX < -0.1)
        {
            UpdateDiffPos(-diffSpeed*runtime.seconds());
        }
    }

    private void UpdateDiffPos(double delta)
    {
        if ((localWristPos + delta >= 0 && localWristPos + delta <= 1))
        {
            localWristPos += delta;
        }
        cs.SpecifyDiffPos(localWristPos);
        cs.SetDiffPos(localWristPos);
    }

    private void TelemetryPrint()
    {
        telemetry.addData("counting telemetry", shark.countingTelemetry);
        telemetry.addData("local offset x", localOffset);

        telemetry.addData("error x", shark.GetErrorX());
        telemetry.addData("error y", shark.GetErrorY());

        telemetry.addData("output x", shark.GetOutputX());
        telemetry.addData("output y", shark.GetOutputY());

        telemetry.addData("x position", shark.GetPositionX());
        telemetry.addData("y position", shark.GetPositionY());
        telemetry.addData("h position", shark.GetImuReading());
        telemetry.addData("orientation", shark.GetOrientation());
        telemetry.addData("last valid imu reading", shark.GetLastValidIMUReading());

//        telemetry.addData("raw localization x", shark.GetLocalizationX());
//        telemetry.addData("raw localization y", shark.GetLocalizationY());
//        telemetry.addData("cam is valid", shark.CamIsValid());
        telemetry.addData("ODO pos", shark.PrintOdometryLocalization());
//        telemetry.addData("leftWrist", cs.GetWristLPosition());
//        telemetry.addData("rightWrist",cs.GetWristRPosition());
        telemetry.addData("target extension", extendPos);
        telemetry.addData("arm position", am.GetCurrentPosition());
//        telemetry.addData("ERROR", am.GetError());
//        telemetry.addData("PROPORTIONAL", am.GetProportional());
//        telemetry.addData("INTEGRAL", am.GetIntegral());
//        telemetry.addData("DERIVATIVE", am.GetDerivative());
//        telemetry.addData("SETPOINT", am.GetSetpoint());
//        telemetry.addData("localL", localDiffL);
//        telemetry.addData("localR", localDiffR);
//        telemetry.addData("INTEGRAL X", shark.GetIntegralSumX());
//        telemetry.addData("INTEGRAL Y", shark.GetIntegralSumY());
//        telemetry.addData("DiagonalScalar", shark.GetDiagonalScalar());
//        telemetry.addData("cv dist", cv.GetDistance());
//        telemetry.addData("cv cx", cv.GetCX());
//        telemetry.addData("cv xOffset", cv.GetXOffset());
        telemetry.addData("x position", shark.GetPositionX());

        telemetry.addData("errorX", shark.GetErrorX());
        telemetry.addData("errorY", shark.GetErrorY());
        telemetry.addData("autograb zerox", shark.GetAutograbZeroX());
        telemetry.addData("autograb zeroy", shark.GetAutograbZeroY());
        telemetry.addData("localWristPos", localWristPos);
//        telemetry.addData("cv points", cv.GetPoints());
//        telemetry.addData("xMin", cv.GetXMin());
//        telemetry.addData("xMax", cv.GetXMax());
//        telemetry.addData("minDiff", cv.GetMinDiff());
//        telemetry.addData("maxDiff", cv.GetMaxDiff());
//        telemetry.addData("yMin", cv.GetYMin());
//        telemetry.addData("yMinX", cv.GetYMinX());
//        telemetry.addData("theta", cv.GetTheta());
//        telemetry.addData("cv detecting", cv.GetDetecting());
//        telemetry.addData("using wrist", moveCmd.UsingWrist());
        telemetry.addData("local roll position", extend.GetRollLocalPosition());
        telemetry.addData("local pitch position", extend.GetPitchLocalPosition());
        telemetry.addData("shark compelted?", shark.GetBoolsCompleted());
        telemetry.addData("command completed?", moveCmd.GetCommandState());
//        telemetry.addData("wrapped theta", cv.GetWrappedTheta());
        telemetry.update();
    }
}

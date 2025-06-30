package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@Config
@TeleOp
public class StarterBot extends LinearOpMode{
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    SharkDrive shark = new SharkDrive();
    MoveCommand moveCmd = new MoveCommand();
    ElapsedTime runtime = new ElapsedTime();
    opencv cv = new opencv();
    Extension extend = new Extension();

    boolean canShift = true;
    double currentSpeed = 1.0;
    double speedInterval = 0.4;

    double diffSpeed = 1;

    double localOffset = 4;
    double localOffsetIncrement = 4;
    double grabDistance = 7; // 4 with proper wall
    double hookDistance = 23.5;

    boolean canShiftArm = true;

    boolean normalControl = true;


    boolean canStartSubMacro = true;
    boolean canStartGrabMacro = true;
    boolean canStartHookMacro = true;
    double localDiffL = 0.44;
    double localDiffR = 0.99;

    double preciseLenience = 0.6;
    double arcLenience = 10;

    double xLocalRehomePos = 0;

    double yLocalRehomePos = 0;

    double xRehomePos = 0; // 2
    double yRehomePos = 0; // 31.5

    boolean canGrab = false;
    boolean canSetDiffPos = true;

    double extensionRollSpeed = 1;

    int hookHeight = 1250;
    int grabHeight = 120;
    int extensionSafetyThreshold = -200;

    // inches
    double extendLength = 19.5;

    int extendPos = 0;

    /*
       ANGLED APPROACH
        arm: 1300
        L: 0.854
        R: 0.656
     */

    /*
    for every specimen, 1 inch offset on the x axis and a 0.3 inch on y
     */

    @Override
    public void runOpMode()
    {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        dt.init(hardwareMap);
        cs.init(hardwareMap, false);
        am.init(hardwareMap);
        shark.init(hardwareMap, false);
        moveCmd.init(hardwareMap, false);
        cv.init(hardwareMap);
        extend.init(hardwareMap);

        waitForStart();
        am.SetArmSpeed(0.45);
        cs.SetDiffPos(0,0);
        am.Rotate(0,'T');
        am.SetLocalNeutral(50);
        am.Rotate(0,'T');

        cv.StartStream(telemetry);
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

                dt.FieldOrientedTranslate(targetPowerX,targetPowerY,targetRotation, shark.GetImuReading());
                ResetLocalOffset(dpadUpPressed);
                SpeedShift(leftBumperPressed,rightBumperPressed);
//
                Rehoming(dpadDownPressed);
//
                ResetTopEncoder(yButtonPressed2);
                ResetDownEncoder(xButtonPressed2);
                ArmSpeedToggle(armToggle);
                ClawControl(clawClose,clawOpen);
                ExtensionClawControl(clawClose, clawOpen);

                ExtensionControl(dpadUpPressed2,dpadDownPressed2);
                ExtensionManipulatorControl(dpadRightPressed2,dpadLeftPressed2,aButtonPressed2,bButtonPressed2);

                if (!canGrab)
                {
                    ArmControl(-armInput);
                }
                DiffControl(wristInputX, wristInputY);

                extend.Update();
            }

            /*
            hold button that make it go arm down and then when release go down
             */

            runtime.reset();
            SubmersibleGrab(aButtonPressed);
            HookMacro(yButtonPressed);
            GrabMacro(bButtonPressed);
            cs.Update();
        }
        cv.StopStream();
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

    private void SubmersibleGrab(boolean a)
    {
        if (a && canStartSubMacro)
        {
            normalControl = false;
            canStartSubMacro = false;
            cv.SetDetecting(true);

//            TeleopMoveCommandA(1, 0, 32, 0,preciseLenience,2,1,grabHeight,0,0,0, true,'G', false);
            TeleopMoveCommandCV(1, 0, 0, 0,preciseLenience,2,1,grabHeight, false, false, 'm');
            cv.SetDetecting(false);
            TeleopMoveCommandCV(0, 0, 0, 0,preciseLenience,2,1,grabHeight, false, false, 'e');
            TeleopMoveCommandCV(0, 0, 0, 0,preciseLenience,2,1,grabHeight, false, false, 'w');
            TeleopMoveCommandCV(0, 0, 0, 0,preciseLenience,2,1,grabHeight, false, true, 'c');

            am.SetLocalNeutral(am.GetTargetPosition());
            normalControl = true;
        }
        else if (!a)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartSubMacro = true;
        }
    }

    private void HookMacro(boolean y)
    {
        if (y && canStartHookMacro)
        {
            normalControl = false;
            canStartHookMacro = false;

            TeleopMoveCommandY(1, 40, grabDistance + 2, 0,preciseLenience,2,0.7,hookHeight,0, 0,0,true, 'B', false);
            TeleopMoveCommandY(1, 0 + localOffset, grabDistance + 2, 0,preciseLenience,0,1,hookHeight,0, 0,0,true, 'B', false);

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

            TeleopMoveCommandB(1, 40, grabDistance + 4, 0,1,2,1, grabHeight, 0,0,0, false, '/', false);
            TeleopMoveCommandB(1, 40, grabDistance, 0,1,2,1, grabHeight, 0,0,0, true, '/', false);

            am.SetLocalNeutral(am.GetTargetPosition());
            normalControl = true;
        }
        else if (!b)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartGrabMacro = true;
        }
    }

    private void TeleopMoveCommandCV(double speed, double x, double y, double h, double d, int axis, double speedA, int a, boolean claw, boolean extendClaw, char type)
    {
        do
        {
//            TelemetryPrint();
            cs.SetClawOpen(claw);
            TelemetryPrint();

            // set extension to max
            int e = -1500;

            //  convert extension into 0-1 scale
            double multiplier = cv.GetDistance() / extendLength;

            if (multiplier > 1) { multiplier = 1; }

            e *= multiplier;

            extendPos = e;

            // checks if the object is within reach, if it is do not move
            double correctDist = extendLength - cv.GetDistance();
            if (correctDist > 3)
            {
                correctDist = 0;
            }

            // 180 deg = 0.7 L 0.8 R
            double theta = cv.GetTheta();

            double rollWrist = 1;
            double pitchWrist = 0.33; // 0.15 offset

//            if (theta > 90)
//            {
//                rollWrist = (theta-90) / 250 + 0;
//            }
//            else
//            {
//                rollWrist = 0.35 + (theta) / 255;
//            }

            moveCmd.MoveToPositionCV(speed,x,y,h,d,axis,speedA,a,e,claw,rollWrist,pitchWrist, cv.GetCX(), correctDist, extendClaw, type);
            if (!gamepad1.a) {
                normalControl = true;
                cv.SetDetecting(false);
                am.SetLocalNeutral(a);
                cv.SetFirstDetection(true);
                return;
            }
        } while (!moveCmd.GetCommandState());
        cv.SetFirstDetection(true);
    }

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

    private void TeleopMoveCommandB(double speed, double x, double y, double h, double d, int axis, double speedA, int a, int e, double roll, double pitch, boolean claw, char wrist, boolean extendClaw)
    {
        do
        {
//            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed,x,y,h,d,axis,speedA,a,e,roll, pitch, claw,wrist,extendClaw);
            if (!gamepad1.b) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

    private void TeleopMoveCommandY(double speed, double x, double y, double h, double d, int axis, double speedA, int a, int e, double roll, double pitch, boolean claw, char wrist, boolean extendClaw)
    {
        do
        {
//            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed,x,y,h,d,axis,speedA,a,e,roll, pitch, claw,wrist,extendClaw);
            if (!gamepad1.y) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

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
            roll += extensionRollSpeed * runtime.milliseconds();
        }
        else if (negativeRoll)
        {
            roll -= extensionRollSpeed * runtime.milliseconds();
        }

        roll = Range.clip(roll,0,1);

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
        if (left > 0.8)
        {
            extend.CloseExtendClaw();
        }
        else if (right > 0.8)
        {
            extend.OpenExtendClaw();
        }
    }

    private void DiffControl(double wristInputX, double wristInputY)
    {
        if (wristInputX > 0.1)
        {
            UpdateDiffPos(-diffSpeed*runtime.seconds(),-diffSpeed*runtime.seconds());
        }
        else if (wristInputX < -0.1)
        {
            UpdateDiffPos(diffSpeed*runtime.seconds(),diffSpeed*runtime.seconds());
        }

        if (wristInputY > 0.1)
        {
            UpdateDiffPos(-diffSpeed*runtime.seconds(),diffSpeed*runtime.seconds());
        }
        else if (wristInputY < -0.1)
        {
            UpdateDiffPos(diffSpeed*runtime.seconds(),-diffSpeed*runtime.seconds());
        }
    }

    private void UpdateDiffPos(double lDelta, double rDelta)
    {
        if ((localDiffL + lDelta >= 0 && localDiffL + lDelta <= 1) && (localDiffR + rDelta >= 0 && localDiffR + rDelta <= 1)) {
            localDiffL += lDelta;
            localDiffR += rDelta;
        }
        cs.SetDiffPos(localDiffL,localDiffR);
    }

    private void TelemetryPrint()
    {
        telemetry.addData("x position", shark.GetPositionX());
        telemetry.addData("y position", shark.GetPositionY());
        telemetry.addData("h position", shark.GetImuReading());
        telemetry.addData("leftWrist", cs.GetWristLPosition());
        telemetry.addData("rightWrist",cs.GetWristRPosition());
        telemetry.addData("target extension", extendPos);
//        telemetry.addData("arm position", am.GetCurrentPosition());
//        telemetry.addData("errorX", shark.GetErrorX());
//        telemetry.addData("errorY", shark.GetErrorY());
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
        telemetry.addData("cv dist", cv.GetDistance());
        telemetry.addData("cv cx", cv.GetCX());
//        telemetry.addData("cv points", cv.GetPoints());
//        telemetry.addData("xMin", cv.GetXMin());
//        telemetry.addData("xMax", cv.GetXMax());
        telemetry.addData("minDiff", cv.GetMinDiff());
        telemetry.addData("maxDiff", cv.GetMaxDiff());
//        telemetry.addData("yMin", cv.GetYMin());
//        telemetry.addData("yMinX", cv.GetYMinX());
        telemetry.addData("theta", cv.GetTheta());
        telemetry.addData("cv detecting", cv.GetDetecting());
        telemetry.addData("using wrist", moveCmd.UsingWrist());
        telemetry.addData("local roll position", extend.GetRollLocalPosition());
        telemetry.addData("local pitch position", extend.GetPitchLocalPosition());
//        telemetry.addData("wrapped theta", cv.GetWrappedTheta());
        telemetry.update();
    }
}

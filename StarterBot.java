package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config
@TeleOp
public class StarterBot extends LinearOpMode{

    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    SharkDrive shark = new SharkDrive();
    MoveCommand moveCmd = new MoveCommand();
    ElapsedTime runtime = new ElapsedTime();

    ElapsedTime ascentDelay = new ElapsedTime();

    opencv cv = new opencv();

    boolean fieldOriented = false;
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

    boolean canStartHookMacro = true;
    boolean canStartGrabMacro = true;
    boolean canStartAscentMacro = true;
    double localDiffL = 0.44;
    double localDiffR = 0.99;


    double runtimeXSum = 0;
    double runtimeXSumScalar = 0.0002; // 0.0164
    double lastX = 0;

    double runtimeYSum = 0;
    double runtimeYSumScalar = -0.006; // 0.028
    double lastY = 0;

    double preciseLenience = 0.6;
    double arcLenience = 10;

    double xLocalRehomePos = 0;

    double yLocalRehomePos = 0;

    double xRehomePos = 2;
    double yRehomePos = 31.5;

    boolean canGrab = false;
    boolean canSetDiffPos = true;

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
        waitForStart();
        // 0.275
        // 0.99
        am.SetArmSpeed(0.45);
        cs.SetDiffPos(0.44,0.99);
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

            double rightTriggerPressed = gamepad1.right_trigger;

            //arm - add encoders later
            double wristInputX = gamepad2.right_stick_x; // usually left
            double wristInputY = gamepad2.right_stick_y;

            double armInput = gamepad2.left_stick_y;

            double clawOpen = gamepad2.left_trigger;
            double clawClose = gamepad2.right_trigger;

            boolean armToggle = gamepad2.left_bumper;

            boolean xButtonPressed2 = gamepad2.x;
            boolean yButtonPressed2 = gamepad2.y;

            boolean middleGrabMacro = gamepad2.right_bumper;
            if (normalControl)
            {
                targetPowerX *= currentSpeed;
                targetPowerY *= currentSpeed;
                targetRotation *= currentSpeed;

                ResetLocalOffset(dpadUpPressed);
//                FieldOrientedToggle(aButtonPressed, xButtonPressed);
                SpeedShift(leftBumperPressed,rightBumperPressed);
                if (fieldOriented)
                {
                    dt.FieldOrientedTranslate(targetPowerX,targetPowerY,targetRotation, shark.GetImuReading());
                }
                else
                {
                    dt.Translate(targetPowerX, targetPowerY, targetRotation);
                }

                Rehoming(dpadDownPressed);

                ResetTopEncoder(yButtonPressed2);
                ResetDownEncoder(xButtonPressed2);
                ArmSpeedToggle(armToggle);
                ClawControl(clawClose,clawOpen);
                if (!canGrab)
                {
                    ArmControl(-armInput);
                }
                DiffControl(wristInputX, wristInputY);
                MiddleGrabMacro(middleGrabMacro);
            }

            /*
            hold button that make it go arm down and then when release go down
             */

            runtime.reset();
            HookMacro(yButtonPressed);
            GrabMacro(bButtonPressed);
            SampleAuto(aButtonPressed);
//            AscentMacro(rightTriggerPressed);
            Integrals();

            cs.Update();

            TelemetryPrint();
        }
        cv.StopStream();
    }

    private void Rehoming(boolean pressed)
    {
        if (!pressed) {return;}

        shark.Rehome();
        xLocalRehomePos = xRehomePos;
        yLocalRehomePos = yRehomePos;
        runtimeXSum = 0;
        runtimeYSum = 0;
    }

    private void Integrals()
    {
        runtimeXSum += Math.abs(shark.GetPositionX() - lastX);
        lastX = shark.GetPositionX();

        runtimeYSum += Math.abs(shark.GetPositionY() - lastY);
        lastY = shark.GetPositionY();
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

    private void FieldOrientedToggle(boolean a, boolean x)
    {
        if (a)
        {
            fieldOriented = true;
        }
        else if (x)
        {
            fieldOriented = false;
        }
    }

    private void TelemetryPrint()
    {
        telemetry.addData("x position", shark.GetPositionX());
        telemetry.addData("y position", shark.GetPositionY());
        telemetry.addData("h position", shark.GetImuReading());
        telemetry.addData("leftWrist", cs.GetWristLPosition());
        telemetry.addData("rightWrist",cs.GetWristRPosition());
        telemetry.addData("arm position", am.GetCurrentPosition());
        telemetry.addData("errorX", shark.GetErrorX());
        telemetry.addData("errorY", shark.GetErrorY());
//        telemetry.addData("ERROR", am.GetError());
//        telemetry.addData("PROPORTIONAL", am.GetProportional());
//        telemetry.addData("INTEGRAL", am.GetIntegral());
//        telemetry.addData("DERIVATIVE", am.GetDerivative());
//        telemetry.addData("SETPOINT", am.GetSetpoint());
        telemetry.addData("localL", localDiffL);
        telemetry.addData("localR", localDiffR);
        telemetry.addData("INTEGRAL X", shark.GetIntegralSumX());
        telemetry.addData("INTEGRAL Y", shark.GetIntegralSumY());
        telemetry.addData("DiagonalScalar", shark.GetDiagonalScalar());
        telemetry.addData("cv dist", cv.GetDistance());
        telemetry.addData("cv cx", cv.GetCX());
//        telemetry.addData("Largest Contour", cv.GetBigContour());
        telemetry.addData("theta", cv.GetTheta());
        telemetry.update();
    }

    private void HookMacro(boolean y)
    {
        if (y && canStartHookMacro)
        {
            normalControl = false;
// should be 23 for move in y
            TeleopMoveCommandY(1, 40, grabDistance + 2, 0,preciseLenience,2,0.7,1250, true, 'B');
            TeleopMoveCommandY(1, 0 + localOffset, grabDistance + 2, 0,preciseLenience,0,1,1250, true, 'B');
//            TeleopMoveCommandY(0.8, 0 + localOffset, hookDistance, 0,preciseLenience,1,1, 1250, true, 'B');
//            TeleopMoveCommandY(0.8, 0 + localOffset - localOffsetIncrement, hookDistance, 0,preciseLenience,0,0.5, 1250, true, 'B');

//            TeleopMoveCommandY(0.5, 0 + localOffset, 23.5, 0,preciseLenience,1,0.5, 1275, true, 'B');
//            if (gamepad1.y)
//            {
//                dt.FieldOrientedTranslate(-1,0,0,s1.GetImuReading());
////            s1.OdometryControl(0,0 + localOffset, 23, 0, arcLenience);
//                cs.SetClawOpen(false);
//                cs.Update();
//                sleep(150);
//                dt.FieldOrientedTranslate(0,0,0,0);
//                cs.Update();
//                cs.SetWristMode('S');
//                sleep(150);
//            }
//            else
//            {
//                normalControl = true;
//                return;
//            }
//            TeleopMoveCommandY(1, 0 + localOffset, 14, 0, preciseLenience,1,0.5,1250, false, 'S');
//            localOffset -= localOffsetIncrement;
            am.SetLocalNeutral(am.GetTargetPosition());
            normalControl = true;
            canStartHookMacro = false;
        }
        else if (!y)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartHookMacro = true;
        }
    }

    private void SampleAuto(boolean a)
    {
        if (a && canStartHookMacro)
        {
//            TeleopMoveCommandA(0.3,340,10.5,0,1,0, 0.4,1625,false,'N');
            TeleopMoveCommandA(0.3,340,12,0,1,2, 0.4,1625,false);
            TeleopMoveCommandA(0,340,13,0,1000,2,0.4,1675,false);
            TeleopMoveCommandA(0,340,13,0,1000,2,0.4,1675,true);
            sleep(500);
            TeleopMoveCommandA(0,340,1,0,1000,2,0.4,1500,true);
        }
        else if (!a)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartGrabMacro = true;
        }
    }

    private void GrabMacro(boolean b)
    {
//        if (gamepad1.b)
//        {
//            TeleopMoveCommandB(0.2,340,getDistance(width),0,0.5,2,0.4,1625,false,'T');
//            TeleopMoveCommandB(0.2,340,getDistance(width),0,0.5,2,0.4,1725,false,'T');
//            TeleopMoveCommandB(0.2,340,getDistance(width),0,0.5,2,0.4,1725,true,'T');
//        }


        if (b && canStartGrabMacro)
        {
//            TeleopMoveCommandB(1, 40, grabDistance + 4, 0,preciseLenience,1,0.5, 190, false, 'G');\
            TeleopMoveCommandB(1, 40, grabDistance + 4, 0,1,2,1, 120, false, 'G');
//            TeleopMoveCommandB(0.6, 40, grabDistance, 0,preciseLenience, 2,0.5,120, false, 'G');
//
//            if (gamepad1.b)
//            {
//                dt.FieldOrientedTranslate(0,0,0, shark.GetImuReading());
//                sleep(500);
//                dt.FieldOrientedTranslate(0,-0.3,0, shark.GetImuReading());
//                sleep(300);
//                dt.FieldOrientedTranslate(0,0,0,0);
//                cs.SetClawOpen(true);
//                cs.Update();
//                sleep(300);
//            }
//            else
//            {
//                normalControl = true;
//                return;
//            }
//            TeleopMoveCommandB(0, 40, grabDistance, 0,arcLenience, 2,1,250, true, 'G');
//            canStartGrabMacro = false;
            am.SetLocalNeutral(am.GetTargetPosition());
        }
        else if (!b)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartGrabMacro = true;
        }
    }

    private void TeleopMoveCommandA(double speed, double x, double y, double h, double d, int axis, double speedA, int a, boolean claw)
    {
        do
        {
            Integrals();
            TelemetryPrint();
            cs.SetClawOpen(claw);
            double wristL = cv.GetTheta() / 250 + 0.12;
            double wristR = cv.GetTheta() / 250 + 0.61;
            moveCmd.MoveToPositionCV(speed,x,y,h,d,axis,speedA,a,claw,wristL,wristR, cv.GetCX(), cv.GetDistance());
            if (!gamepad1.a) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

    private void TeleopMoveCommandB(double speed, double x, double y, double h, double d, int axis, double speedA, int a, boolean claw, char wrist)
    {
        do
        {
            Integrals();
//            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed,x + runtimeXSum * runtimeXSumScalar - xLocalRehomePos,y + runtimeYSum * runtimeYSumScalar - yLocalRehomePos,h,d,axis,speedA,a,claw,wrist);
            if (!gamepad1.b) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

    private void TeleopMoveCommandY(double speed, double x, double y, double h, double d, int axis, double speedA, int a, boolean claw, char wrist)
    {
        do
        {
            Integrals();
//            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed,x + runtimeXSum * runtimeXSumScalar - xLocalRehomePos,y + runtimeYSum * runtimeYSumScalar - yLocalRehomePos,h,d,axis,speedA,a,claw,wrist);
            if (!gamepad1.y) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

    private void MiddleGrabMacro(boolean input)
    {
        if (input && !canGrab)
        {
            canGrab = true;
            am.SetArmSpeed(1);
            am.Rotate(1625,'A');
            if (canSetDiffPos)
            {
                localDiffL = 0.67;
                localDiffR = 0.83;
                cs.SetDiffPos(localDiffL,localDiffR);
                canSetDiffPos = false;
            }
            cs.SetClawOpen(false);
            cs.Update();
        }
        else if (!input && canGrab)
        {
            canSetDiffPos = true;
            am.SetArmSpeed(1);
            am.Rotate(1725, 'A');
            sleep(500);
            am.Rotate(1725, 'A');
            cs.SetClawOpen(true);
            cs.Update();
            sleep(750);
            am.Rotate(1650, 'A');
            localDiffL = 0.9;
            localDiffR = 0.5;
            cs.SetDiffPos(localDiffL,localDiffR);
            cs.Update();
            am.SetArmSpeed(0.45);
            canGrab = false;
        }
    }

    private void ArmSpeedToggle(boolean armToggle)
    {
        if (armToggle)
        {
            if (!canShiftArm) { return; }

            if (am.GetArmSpeed() == 0.45)
            {
                am.SetArmSpeed(0.15);
                canShiftArm = false;
            }
            else if (am.GetArmSpeed() == 0.15)
            {
                am.SetArmSpeed(0.45);
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
}

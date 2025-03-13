package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;

//@Config
@TeleOp
public class StarterBot extends LinearOpMode{
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    OdometrySensor s1 = new OdometrySensor();
    MoveCommand moveCmd = new MoveCommand();
    ElapsedTime runtime = new ElapsedTime();

    ElapsedTime ascentDelay = new ElapsedTime();

    boolean fieldOriented = false;
    boolean canShift = true;
    double currentSpeed = 1.0;
    double speedInterval = 0.4;

    double diffSpeed = 0.5;

    double localOffset = 4;
    double localOffsetIncrement = 4;
    double grabDistance = 7; // 4 with proper wall
    double hookDistance = 24;

    boolean canShiftArm = true;

    boolean normalControl = true;

    boolean canStartHookMacro = true;
    boolean canStartGrabMacro = true;
    boolean canStartAscentMacro = true;
    double localDiffL = 0.44;
    double localDiffR = 0.99;


    double runtimeXSum = 0;
    double runtimeXSumScalar = 0.0156; // 0.0164
    double lastX = 0;

    double runtimeYSum = 0;
    double runtimeYSumScalar = 0.0063; // 0.028
    double lastY = 0;

    double preciseLenience = 0.6;
    double arcLenience = 10;

    double xLocalRehomePos = 0;

    double yLocalRehomePos = 0;

    double xRehomePos = 2;
    double yRehomePos = 31.5;

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
        s1.init(hardwareMap, false);
        moveCmd.init(hardwareMap, false);
        waitForStart();
        // 0.275
        // 0.99
        cs.SetDiffPos(0.44,0.99);
        am.Rotate(0,'T');
        am.SetLocalNeutral(50);
        am.Rotate(0,'T');
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
            double wristInputX = gamepad2.left_stick_x;
            double wristInputY = gamepad2.right_stick_y;

            boolean armInputUp = gamepad2.dpad_up;
            boolean armInputDown = gamepad2.dpad_down;

            double clawOpen = gamepad2.left_trigger;
            double clawClose = gamepad2.right_trigger;

            boolean armToggle = gamepad2.left_bumper;

            boolean xButtonPressed2 = gamepad2.x;
            boolean yButtonPressed2 = gamepad2.y;

            boolean middleGrabMacro = gamepad2.right_bumper;

            if (normalControl)
            {
                ResetLocalOffset(dpadUpPressed);
                FieldOrientedToggle(aButtonPressed, xButtonPressed);
                SpeedShift(leftBumperPressed,rightBumperPressed);
                dt.FieldOrientedTranslate(targetPowerX,targetPowerY,targetRotation, s1.GetImuReading());
                Rehoming(dpadDownPressed);

                ResetTopEncoder(yButtonPressed2);
                ResetDownEncoder(xButtonPressed2);
                ArmSpeedToggle(armToggle);
                ClawControl(clawOpen,clawClose);
                ArmControl(armInputUp,armInputDown);
                DiffControl(wristInputX, wristInputY);
            }

            runtime.reset();

            MiddleGrabMacro(middleGrabMacro);
            HookMacro(yButtonPressed);
            GrabMacro(bButtonPressed);
            AscentMacro(rightTriggerPressed);
            Integrals();

            cs.Update();

            TelemetryPrint();
        }
    }

    /* hook dimensions
    x - 0 inches
    y - 28.5 inches
    h - 0 degrees
    wristL = 1
    wristR = 0.555
    armPosition - -4650
     */

    private void Rehoming(boolean pressed)
    {
        if (!pressed) {return;}

        s1.Rehome();
        xLocalRehomePos = xRehomePos;
        yLocalRehomePos = yRehomePos;
        runtimeXSum = 0;
        runtimeYSum = 0;
    }

    private void Integrals()
    {
        runtimeXSum += Math.abs(s1.GetPositionX() - lastX);
        lastX = s1.GetPositionX();

        runtimeYSum += Math.abs(s1.GetPositionY() - lastY);
        lastY = s1.GetPositionY();
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
        telemetry.addData("x position", s1.GetPositionX());
        telemetry.addData("y position", s1.GetPositionY());
        telemetry.addData("h position", s1.GetImuReading());
        telemetry.addData("leftWrist", cs.GetWristLPosition());
        telemetry.addData("rightWrist",cs.GetWristRPosition());
        telemetry.addData("arm position", am.GetCurrentPosition());
        telemetry.addData("errorX", s1.GetErrorX());
        telemetry.addData("errorY", s1.GetErrorY());
//        telemetry.addData("ERROR", am.GetError());
//        telemetry.addData("PROPORTIONAL", am.GetProportional());
//        telemetry.addData("INTEGRAL", am.GetIntegral());
//        telemetry.addData("DERIVATIVE", am.GetDerivative());
//        telemetry.addData("SETPOINT", am.GetSetpoint());
        telemetry.addData("localL", localDiffL);
        telemetry.addData("localR", localDiffR);
        telemetry.addData("INTEGRAL X",s1.GetIntegralSumX());
        telemetry.addData("INTEGRAL Y", s1.GetIntegralSumY());
        telemetry.addData("DiagonalScalar", s1.GetDiagonalScalar());
        telemetry.update();
    }

    /*
    -29.5, 47, -90, 1175
    0 while moving x >
     */

    private void AscentMacro(double input)
    {
        if (input > 0.8 && canStartAscentMacro)
        {
            normalControl = false;
            if (ascentDelay.seconds() > 0) // curren\\.tly unimplemented
            {
                TeleopMoveCommandRT(1,-40,10,0,arcLenience,2,0.5,500,false,'/');
                TeleopMoveCommandRT(1,-40,50,0,preciseLenience,1,0.5,500,false,'/');
                TeleopMoveCommandRT(1,-40,50,-90,preciseLenience,2,0.5,500,false,'/');
                TeleopMoveCommandRT(1,-28,50,-90,preciseLenience,0,0.5,500,false,'/');
                TeleopMoveCommandRT(1,-26,50,-90,preciseLenience,0,1,1175, false,'/');
//                TeleopMoveCommandRT(1,-27,47,-90,1175,false,'/');
//                TeleopMoveCommandRT(1,-20,47,-90,1175 ,false,'/');
//                TeleopMoveCommandRT(1,-20,47,0,0,false,'/');
                if (gamepad1.right_trigger < 0.8)
                {
                    normalControl = true;
                    return;
                }
                dt.FieldOrientedTranslate(-1,0,0,s1.GetImuReading());
                sleep(500);
                while (true)
                {
                    dt.FieldOrientedTranslate(-1,0,0,s1.GetImuReading());
                    am.Rotate(0,'A');
                    sleep(2000);
                    while (true)
                    {
                        dt.FieldOrientedTranslate(0,0,0,0);
                        am.Rotate(0,'A');
                    }
                }
            }
            canStartAscentMacro = false;
        }
        else if (input < 0.8)
        {
            canStartAscentMacro = true;
            ascentDelay.reset();
        }
    }

    /*
    1
    6.5 x
    -2 y

    2
    6 x
    -2.9 y

    3
    7 x
    -1.7 y
     */

    // hook slide
    private void HookMacro(boolean y)
    {
        if (y && canStartHookMacro)
        {
            normalControl = false;
// should be 23 for move in y
            TeleopMoveCommandY(1, 40, grabDistance + 2, 0,preciseLenience,2,0.25,1250, true, 'B');
            TeleopMoveCommandY(1, 0 + localOffset, grabDistance + 2, 0,preciseLenience,0,0.25,1250, true, 'B');
            TeleopMoveCommandY(0.8, 0 + localOffset, hookDistance, 0,preciseLenience,1,0.5, 1250, true, 'B');
//            TeleopMoveCommandY(0.8, 0 + localOffset - localOffsetIncrement, hookDistance, 0,preciseLenience,0,0.5, 1250, true, 'B');

//            TeleopMoveCommandY(0.5, 0 + localOffset, 23.5, 0,preciseLenience,1,0.5, 1275, true, 'B');
            if (gamepad1.y)
            {
                dt.FieldOrientedTranslate(-1,0,0,0);
//            s1.OdometryControl(0,0 + localOffset, 23, 0, arcLenience);
                cs.SetClawOpen(false);
                cs.Update();
                sleep(150);
                dt.FieldOrientedTranslate(0,0,0,0);
                cs.Update();
                cs.SetWristMode('S');
                sleep(150);
            }
            else
            {
                normalControl = true;
                return;
            }
            TeleopMoveCommandY(1, 0 + localOffset, 14, 0, preciseLenience,1,0.5,1250, false, 'S');
//            localOffset -= localOffsetIncrement;
        }
        else if (!y)
        {
            canStartHookMacro = true;
        }
    }

    private void GrabMacro(boolean b)
    {
        if (b && canStartGrabMacro)
        {
//            TeleopMoveCommandB(1, 40, grabDistance + 4, 0,preciseLenience,1,0.5, 190, false, 'G');\
            TeleopMoveCommandB(1, 40, grabDistance + 4, 0,1,2,0.5, 145, false, 'G');
            TeleopMoveCommandB(0.6, 40, grabDistance, 0,preciseLenience, 2,0.5,145, false, 'G');
            TeleopMoveCommandB(0.35, 40, grabDistance, 0,arcLenience, 2,0.5,145, true, 'G');
            if (gamepad1.b)
            {
//            s1.OdometryControl(0,40, grabDistance, 0,arcLenience);
                dt.FieldOrientedTranslate(0,0,0,0);
                cs.SetClawOpen(true);
                sleep(200);
            }
            else
            {
                normalControl = true;
                return;
            }
            TeleopMoveCommandB(0.5, 40, grabDistance,0,preciseLenience,1,1,1250, true, 'G');
//            TeleopMoveCommandB(1, 40, grabDistance + 4,0,preciseLenience,1,1,1275, true, 'G');
            canStartGrabMacro = false;
        }
        else if (!b)
        {
            canStartGrabMacro = true;
        }
    }

    private void TeleopMoveCommandB(double speed, double x, double y, double h, double d, int axis, double speedA, int a, boolean claw, char wrist)
    {
        do
        {
            Integrals();
            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed,x + runtimeXSum * runtimeXSumScalar - xLocalRehomePos,y - runtimeYSum * runtimeYSumScalar - yLocalRehomePos,h,d,axis,speedA,a,claw,wrist);
            if (!gamepad1.b) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
        am.SetLocalNeutral(a);
    }

    private void TeleopMoveCommandY(double speed, double x, double y, double h, double d, int axis, double speedA, int a, boolean claw, char wrist)
    {
        do
        {
            Integrals();
            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed,x + runtimeXSum * runtimeXSumScalar - xLocalRehomePos,y - runtimeYSum * runtimeYSumScalar - yLocalRehomePos,h,d,axis,speedA,a,claw,wrist);
            if (!gamepad1.y) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
        am.SetLocalNeutral(a);
    }

    private void TeleopMoveCommandRT(double speed, double x, double y, double h, double d, int axis, double speedA, int a, boolean claw, char wrist)
    {
        do
        {
            Integrals();
            TelemetryPrint();
            moveCmd.MoveToPositionCancellable(speed,x + runtimeXSum * runtimeXSumScalar - xLocalRehomePos,y - runtimeYSum * runtimeYSumScalar - yLocalRehomePos,h,d,axis,speedA,a,claw,wrist);
            TelemetryPrint();
            if (gamepad1.right_trigger < 0.8) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
        am.SetLocalNeutral(a);
    }

    private void MiddleGrabMacro(boolean input)
    {
        if (input)
        {
            am.Rotate(1600,'A');
            cs.SetWristMode('M');
            normalControl = false;
        }
        else
        {
            normalControl = true;
        }
    }

    private void ArmSpeedToggle(boolean armToggle)
    {
        if (armToggle)
        {
            if (!canShiftArm) { return; }

            if (am.GetArmSpeed() == 1)
            {
                am.SetArmSpeed(0.5);
                canShiftArm = false;
            }
            else if (am.GetArmSpeed() == 0.5)
            {
                am.SetArmSpeed(1);
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

    private void ArmControl(boolean up, boolean down)
    {
        if (up)
        {
            am.Rotate(1,'T');
        }
        else if (down)
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
        else if (wristInputY > 0.1)
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

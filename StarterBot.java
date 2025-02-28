package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
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

    double localOffset = 0;
    double localOffsetIncrement = 1;
    double grabDistance = 5.5;

    boolean canShiftArm = true;

    boolean normalControl = true;

    boolean canStartHookMacro = true;
    boolean canStartGrabMacro = true;
    boolean canStartAscentMacro = true;

    double localDiffL = 0.73;
    double localDiffR = 0.73;


    @Override
    public void runOpMode()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        dt.init(hardwareMap);
        cs.init(hardwareMap, false);
        am.init(hardwareMap);
        s1.init(hardwareMap, false);
        moveCmd.init(hardwareMap, false);
        waitForStart();
        cs.SetDiffPos(0.73,0.73);
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
                dt.FieldOrientedTranslate(targetPowerX / 3,targetPowerY / 3,targetRotation / 3, s1.GetImuReading());

                ResetTopEncoder(yButtonPressed2);
                ResetDownEncoder(xButtonPressed2);
//                ArmSpeedToggle(armToggle);
                ClawControl(clawClose,clawOpen);
                ArmControl(armInputUp,armInputDown);
                DiffControl(wristInputX, wristInputY);
            }

            runtime.reset();

            MiddleGrabMacro(middleGrabMacro);
            HookMacro(yButtonPressed);
            GrabMacro(bButtonPressed);
            AscentMacro(rightTriggerPressed);

            cs.Update();

            telemetry.addData("x position", s1.GetPositionX());
            telemetry.addData("y position", s1.GetPositionY());
            telemetry.addData("h position", s1.GetImuReading());
            telemetry.addData("leftWrist", cs.GetWristLPosition());
            telemetry.addData("rightWrist",cs.GetWristRPosition());
            telemetry.addData("arm position", am.GetCurrentPosition());
            telemetry.addData("localL", localDiffL);
            telemetry.addData("localR", localDiffR);
            telemetry.update();
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
            localOffset = 12;
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

    private void AscentMacro(double input)
    {
        if (input > 0.8 && canStartAscentMacro)
        {
            normalControl = false;
            if (ascentDelay.seconds() > 0) // currently unimplemented
            {
                TeleopMoveCommandRT(1,-44,10,0,-2000,false,'/');
                TeleopMoveCommandRT(1,-44,56,0,-2000,false,'/');
                TeleopMoveCommandRT(1,-44,56,0,-4000,false,'/');
                TeleopMoveCommandRT(1,-44,56,-90,-4000,false,'/');
                TeleopMoveCommandRT(1,-38,56,0,-4000,false,'/');
                TeleopMoveCommandRT(1,-38,56,0,0,false,'/');
            }
            canStartAscentMacro = false;
        }
        else if (input < 0.8)
        {
            canStartAscentMacro = true;
            ascentDelay.reset();
        }
    }

    private void HookMacro(boolean y)
    {
        if (y && canStartHookMacro)
        {
            normalControl = false;

            TeleopMoveCommandY(1, 0 + localOffset, 16, 0, -4730, true, 'B');
            TeleopMoveCommandY(1, 0 + localOffset, 26, 0, -4730, true, 'B');
            TeleopMoveCommandY(1, 0 + localOffset, 20, 0, -4730, false, 'B');

            localOffset -= localOffsetIncrement;
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
            TeleopMoveCommandB(1, 42, grabDistance, 0, -600, false, 'G');
            TeleopMoveCommandB(1, 42, grabDistance, 0, -600, true, 'G');
            TeleopMoveCommandB(1, 42, grabDistance,0, -900, true, 'G');

            canStartGrabMacro = false;
        }
        else if (!b)
        {
            canStartGrabMacro = true;
        }
    }

    private void TeleopMoveCommandB(double speed, double x, double y, double h, int a, boolean claw, char wrist)
    {
        do
        {
            moveCmd.MoveToPositionCancellable(speed,x,y,h,a,claw,wrist);
            if (!gamepad1.b) {
                normalControl = true;
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

    private void TeleopMoveCommandY(double speed, double x, double y, double h, int a, boolean claw, char wrist)
    {
        do
        {
            moveCmd.MoveToPositionCancellable(speed,x,y,h,a,claw,wrist);
            if (!gamepad1.y) {
                normalControl = true;
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

    private void TeleopMoveCommandRT(double speed, double x, double y, double h, int a, boolean claw, char wrist)
    {
        do
        {
            moveCmd.MoveToPositionCancellable(speed,x,y,h,a,claw,wrist);
            if (gamepad1.right_trigger < 0.8) {
                normalControl = true;
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

    private void MiddleGrabMacro(boolean input)
    {
        if (input)
        {
            am.Rotate(-6500,'A');
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

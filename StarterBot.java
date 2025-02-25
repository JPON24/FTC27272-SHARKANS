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

    boolean fieldOriented = false;
    boolean canShift = true;
    double currentSpeed = 1.0;
    double speedInterval = 0.4;

    double diffSpeed = 1;

    double localOffset = 0;
    double localOffsetIncrement = 1;
    double grabDistance = 2;

    boolean canUpdateDiff = true;
    boolean canShiftArm = true;

    boolean normalControl = true;

    boolean canStartHookMacro = true;
    boolean canStartGrabMacro = true;

    double localDiffL = 0;
    double localDiffR = 0;


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
                dt.FieldOrientedTranslate(targetPowerX,targetPowerY,targetRotation, s1.GetImuReading());

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

            telemetry.addData("x position", s1.GetPositionX());
            telemetry.addData("y position", s1.GetPositionY());
            telemetry.addData("h position", s1.GetImuReading());
            telemetry.addData("leftWrist", cs.GetWristLPosition());
            telemetry.addData("rightWrist",cs.GetWristRPosition());
            telemetry.update();
        }
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
            localOffset = 12;
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

    private void HookMacro(boolean y)
    {
        if (y && canStartHookMacro)
        {
            normalControl = false;

            TeleopMoveCommandY(1, 0 + localOffset, 16, 0, -4525, true, 'B');
            TeleopMoveCommandY(1, 0 + localOffset, 24, 0, -4525, true, 'B');
            TeleopMoveCommandY(1, 0 + localOffset, 16, 0, -4525, true, 'B');

            localOffset -= localOffsetIncrement;
            canStartHookMacro = false;
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
            TeleopMoveCommandB(1, 42, grabDistance, 0, -700, false, 'G');
            TeleopMoveCommandB(1, 42, grabDistance, 0, -700, true, 'G');
            TeleopMoveCommandB(1, 42, grabDistance,0, -1000, true, 'G');

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

    private void MiddleGrabMacro(boolean input)
    {
        if (input)
        {
            am.rotate(-6500,'A');
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
            am.rotate(1,'T');
        }
        else if (down)
        {
            am.rotate(-1,'T');
        }
        else
        {
            am.rotate(0,'T');
        }
    }

    private void DiffControl(double wristInputX, double wristInputY)
    {
        if (wristInputX > 0.1)
        {
            UpdateDiffPos(-diffSpeed*runtime.seconds(),diffSpeed*runtime.seconds());
        }
        else if (wristInputX < -0.1)
        {
            UpdateDiffPos(diffSpeed*runtime.seconds(),-diffSpeed*runtime.seconds());
        }
        else if (wristInputY > 0.1)
        {
            UpdateDiffPos(diffSpeed*runtime.seconds(),diffSpeed*runtime.seconds());
        }
        else if (wristInputY < -0.1)
        {
            UpdateDiffPos(-diffSpeed*runtime.seconds(),-diffSpeed*runtime.seconds());
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;
import org.firstinspires.ftc.teamcode.ClawServo;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Extension_1;
import org.firstinspires.ftc.teamcode.OdometrySensor;
import org.firstinspires.ftc.teamcode.MoveCommand;

@TeleOp
public class StarterBot extends LinearOpMode{
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    Extension_1 e1 = new Extension_1();
    OdometrySensor s1 = new OdometrySensor();
    MoveCommand moveCmd = new MoveCommand();
    ElapsedTime runtime = new ElapsedTime();

    boolean fieldOriented = false;
    boolean canShift = true;
    double currentSpeed = 1.0;
    double speedInterval = 0.4;
    
    boolean canShiftWristType = true;
    boolean normalWristType = false;
    boolean lastDpad2RightState = false;
    
    boolean canShiftArm = true;
    double currentArmSpeed = 0.9;
        
    boolean hookMacroActivated = false;

    @Override
    public void runOpMode()
    {
        double targetPowerX = 0;
        double targetPowerY = 0;
        double targetRotation = 0;
        
        boolean leftBumperPressed = false;
        boolean rightBumperPressed = false;
        boolean dpadUp = false;
        boolean aButtonPressed1 = false;
        boolean xButtonPressed1 = false;
        
        //arm
        boolean xButtonPressed = false;
        boolean aButtonPressed = false;
        boolean bButtonPressed = false;

        boolean yButtonPressed = false;
        double armLiftInput = 0;
        double wristInput = 0;
        
        boolean dpadUp2 = false;
        boolean dpadDown2 = false;
        boolean dpadLeft2 = false;
        boolean dpadRight2 = false;
        boolean leftBumperPressed2 = false;
        boolean rightBumperPressed2 = false;
        
        double leftTrigger2 = 0;
        double rightTrigger2 = 0;

        dt.init(hardwareMap);
        cs.init(hardwareMap);
        e1.init(hardwareMap);
        am.init(hardwareMap);
        s1.init(hardwareMap, true);
        moveCmd.init(hardwareMap, false);
        waitForStart();
        
        while(opModeIsActive())
        {
            // triggers should be extension
            // joystick should be wrist movement
            // swap between normal and custom in teleop for wrist
            
            if (hookMacroActivated)
            {
                HookMacro();
                continue;
            }
            targetPowerX = gamepad1.left_stick_x;
            targetPowerY = -gamepad1.left_stick_y;
            targetRotation = gamepad1.right_stick_x;
            
            leftBumperPressed = gamepad1.left_bumper;
            rightBumperPressed = gamepad1.right_bumper;
            dpadUp = gamepad1.dpad_up;
            aButtonPressed1 = gamepad1.a;
            xButtonPressed1 = gamepad1.x;
            
            //arm
            xButtonPressed = gamepad2.x;
            aButtonPressed = gamepad2.a;
            yButtonPressed = gamepad2.y;
            bButtonPressed = gamepad2.b;
            armLiftInput = -gamepad2.left_stick_y;
            wristInput = -gamepad2.right_stick_y;
            
            dpadUp2 = gamepad2.dpad_up;
            dpadDown2 = gamepad2.dpad_down;
            dpadLeft2 = gamepad2.dpad_left;
            dpadRight2 = gamepad2.dpad_right;
            leftBumperPressed2 = gamepad2.left_bumper;
            rightBumperPressed2 = gamepad2.right_bumper;
            
            leftTrigger2 = gamepad2.left_trigger;
            rightTrigger2 = gamepad2.right_trigger;

            if (dpadRight2 && dpadRight2 != lastDpad2RightState)
            {
                hookMacroActivated = true;
            }
            
            lastDpad2RightState = dpadRight2;
            
            //claw swapper
            if (xButtonPressed)
            {
                cs.clawMove(true);
            }
            else if (aButtonPressed)
            {
                cs.clawMove(false);
            }
            // handles different wrist types based off button presses
            SwapWristType('C',yButtonPressed);
            
            if (cs.GetWristState() == 'C')
            {
                runtime.reset();
                if (wristInput < -0.1)
                {
                    cs.MoveToPosition(cs.GetWristPosition() + 1.75 * runtime.milliseconds());
                }
                else if (wristInput > 0.1)
                {
                    cs.MoveToPosition(cs.GetWristPosition() - 1.75 * runtime.milliseconds());
                }
            }

            cs.Update();
            
            // drive
            if (leftBumperPressed)
            {
                if (canShift && currentSpeed - speedInterval > 0)
                {
                    MoveSpeedShift(-speedInterval);
                }
            }
            else if (rightBumperPressed)
            {
                if (canShift && currentSpeed + speedInterval <= 1)
                {
                    MoveSpeedShift(speedInterval);
                }  
            }
            else
            {
                canShift = true;
            }
            
            if (dpadUp)
            {
                s1.ResetImuReadings();
            }
            
            //arm
            if (leftBumperPressed2)
            {
                if (canShiftArm && currentArmSpeed == 0.9)
                {
                    ArmSpeedShift(0.2);
                    canShiftArm = false;
                }
                else if (canShiftArm && currentArmSpeed == 0.2)
                {
                    ArmSpeedShift(0.9);
                    canShiftArm = false;
                }
            }
            else
            {
                canShiftArm = true;
            }
            
            if(dpadUp2)//reset arm pos
            {
                am.ResetEncoders();
            }
            else if (dpadLeft2)//fix extension pos
            {
                e1.ResetEncoders();
            }

            if (aButtonPressed1)
            {
                fieldOriented = true;
            }
            else if (xButtonPressed1)
            {
                fieldOriented = false;
            }
            if (fieldOriented)
            {
                dt.FieldOrientedTranslate(targetPowerX,targetPowerY,targetRotation, s1.GetImuReading());
            }
            else
            {
                dt.Translate(targetPowerX,targetPowerY,targetRotation);
            }
            
            am.rotate(armLiftInput, 'T');
            
            //set arm height for grabbing samples off wall
            if (bButtonPressed)
            {
                am.MoveToPosition(am.ConvertAngleToEncoder(200));
                cs.setWristMode('C');
                cs.MoveToPosition(0.47);
            }
            
            if (rightBumperPressed2)
            {
                am.MoveToPosition(-1825);
                cs.setWristMode('C');
                cs.MoveToPosition(0.22);
            }
            
            if (rightTrigger2 > 0.5)
            {
                e1.Move(rightTrigger2,700, 'T');
            }
            else if (leftTrigger2 > 0.5)
            {
                e1.Move(-leftTrigger2,0,'T');
            }
            else
            {
                e1.Move(0,e1.GetCurrentPosition(),'T');
            }
            
            telemetry.addData("wrist setting",cs.GetWristState());
            telemetry.addData("wrist position", cs.GetWristPosition());
            telemetry.addData("angle double",am.GetNormalizedArmAngle());
            telemetry.addData("arm encoder", am.GetCurrentPosition());
            telemetry.addData("field oriented", fieldOriented);
            telemetry.update();
        }
    }
    
    private void MoveSpeedShift(double speedInterval)
    {
        currentSpeed += speedInterval;
        dt.SetSpeedScalar(currentSpeed);
        canShift = false;
    }
    
    private void ArmSpeedShift(double speedInterval)
    {
        currentArmSpeed = speedInterval;
        am.SetSpeed(currentArmSpeed);
        canShiftArm = false;
    }
    
    private void SwapWristType(char additionalMode, boolean buttonPressed)
    {
        if (buttonPressed && canShiftWristType)
        {
            canShiftWristType = false;
            if (normalWristType)
            {
                cs.setWristMode(additionalMode);
                normalWristType = false;
            }
            else
            {
                cs.setWristMode('N');
                normalWristType = true;
            }
        }
        else if (!buttonPressed && !canShiftWristType)
        {
            canShiftWristType = true;  
        }
    }

    private void HookMacro()
    {
        CancellableCommand(0,165,true,'D'); //1
        CancellableCommand(0,150,true,'B'); //1
        CancellableCommand(0,165,true,'B'); //1
        hookMacroActivated = false;
    }

    private void CancellableCommand(int tgtE, int tgtA, boolean tgtClaw, char tgtWrist)
    {
        if (!hookMacroActivated)
        {
            return;
        }
        moveCmd.MoveToPositionCancellable(tgtE,tgtA,tgtClaw,tgtWrist);
        while (!moveCmd.GetCommandState())
        {
            // if command deactivated then break out of loop
            if (!hookMacroActivated)
            {
                break;
            }

            moveCmd.MoveToPositionCancellable(tgtE,tgtA,tgtClaw,tgtWrist);
            // deactivate command if dpad right pressed
            if (gamepad2.dpad_right && lastDpad2RightState != gamepad2.dpad_right)
            {
                hookMacroActivated = false;
            }
            
            lastDpad2RightState = gamepad2.dpad_right;
        }
    }
}

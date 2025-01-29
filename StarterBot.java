package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;
import org.firstinspires.ftc.teamcode.ClawServo;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Extension_1;
import org.firstinspires.ftc.teamcode.OdometrySensor;

@TeleOp
public class StarterBot extends LinearOpMode{
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    Extension_1 e1 = new Extension_1();
    OdometrySensor s1 = new OdometrySensor();

    boolean fieldOriented = false;
    boolean canShift = true;
    double currentSpeed = 1.0;
    double speedInterval = 0.4;
    
    boolean canShiftWristType = true;
    boolean canShiftCustomWristPos = true;
    boolean normalWristType = false;
    boolean lastXButtonState = false;
    
    boolean canShiftArm = true;
    double currentArmSpeed = 1.0;
    double armSpeedInterval = 0.8;
        
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
        boolean yButtonPressed = false;
        boolean bButtonPressed = false;
        double armLiftInput = 0;
        double armExtendInput = 0;
        
        boolean dpadUp2 = false;
        boolean dpadDown2 = false;
        boolean dpadLeft2 = false;
        boolean leftBumperPressed2 = false;
        boolean rightBumperPressed2 = false;
        
        double leftTrigger2 = 0;
        double rightTrigger2 = 0;

        dt.init(hardwareMap);
        cs.init(hardwareMap);
        e1.init(hardwareMap);
        am.init(hardwareMap);
        s1.init(hardwareMap);
        waitForStart();
        
        while(opModeIsActive())
        {
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
            armExtendInput = -gamepad2.right_stick_y;
            
            dpadUp2 = gamepad2.dpad_up;
            dpadDown2 = gamepad2.dpad_down;
            dpadLeft2 = gamepad2.dpad_left;
            leftBumperPressed2 = gamepad2.left_bumper;
            rightBumperPressed2 = gamepad2.right_bumper;
            
            leftTrigger2 = gamepad2.left_trigger;
            rightTrigger2 = gamepad2.right_trigger;
            
            //claw swapper
            if (xButtonPressed != lastXButtonState)
            {
                cs.clawMove(!cs.GetClawPosition());
            }
            lastXButtonState = xButtonPressed;
            
            // handles different wrist types based off button presses
            SwapWristType('D',yButtonPressed);
            SwapWristType('B',bButtonPressed);
            SwapWristType('C',aButtonPressed);
            
            if (cs.GetWristState() == 'C')
            {
                // 30 degree shifter with triggers
                if (leftTrigger2 > 0.8 && canShiftCustomWristPos)
                {
                    cs.MoveToPosition(cs.GetWristPosition() - 0.1);
                    canShiftCustomWristPos = false;
                }
                else if (rightTrigger2 > 0.8 && canShiftCustomWristPos)
                {
                    cs.MoveToPosition(cs.GetWristPosition() + 0.1);
                    canShiftCustomWristPos = false;
                }
                else
                {
                    canShiftCustomWristPos = true;
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
                if (canShiftArm && currentArmSpeed - armSpeedInterval > 0)
                {
                    ArmSpeedShift(-armSpeedInterval);
                }
            }
            else if (rightBumperPressed2)
            {
                if (canShiftArm && currentArmSpeed + armSpeedInterval <=1)
                {
                    ArmSpeedShift(armSpeedInterval);
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
            else if (dpadDown2)
            {
                am.ResetEncodersUp();
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
            
            if (armExtendInput > 0.1)
            {
                e1.Move(armExtendInput,600, 'T');
            }
            else if (armExtendInput < -0.1)
            {
                e1.Move(armExtendInput,-125,'T');
            }
            else
            {
                e1.Move(0,e1.GetCurrentPosition(),'T');
            }
            
            telemetry.addData("wrist setting",cs.GetWristState());
            telemetry.addData("wrist position", cs.GetWristPosition());
            telemetry.addData("angle double",am.GetNormalizedArmAngle());
            telemetry.addData("field oriented", fieldOriented);
            telemetry.Update();
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
        currentArmSpeed += speedInterval;
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
        // add code here
    }
}

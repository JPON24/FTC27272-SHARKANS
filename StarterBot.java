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
    boolean normalWristType = false;
    
    boolean canShiftArm = true;
    double currentArmSpeed = 1.0;
    double armSpeedInterval = 0.8;
        
    @Override
    public void runOpMode()
    {
        dt.init(hardwareMap);
        cs.init(hardwareMap);
        e1.init(hardwareMap);
        am.init(hardwareMap);
        s1.init(hardwareMap);
        waitForStart();
        
        while(opModeIsActive())
        {
            double targetPowerX = gamepad1.left_stick_x;
            double targetPowerY = -gamepad1.left_stick_y;
            double targetRotation = gamepad1.right_stick_x;
            
            boolean leftBumperPressed = gamepad1.left_bumper;
            boolean rightBumperPressed = gamepad1.right_bumper;
            boolean dpadUp = gamepad1.dpad_up;
            boolean aButtonPressed1 = gamepad1.a;
            boolean xButtonPressed1 = gamepad1.x;
            
            //arm
            boolean xButtonPressed = gamepad2.x;
            boolean aButtonPressed = gamepad2.a;
            boolean yButtonPressed = gamepad2.y;
            double armLiftInput = -gamepad2.left_stick_y;
            double armExtendInput = -gamepad2.right_stick_y;
            
            boolean dpadUp2 = gamepad2.dpad_up;
            boolean dpadDown2 = gamepad2.dpad_down;
            boolean dpadLeft2 = gamepad2.dpad_left;
            boolean leftBumperPressed2 = gamepad2.left_bumper;
            boolean rightBumperPressed2 = gamepad2.right_bumper;
            
            double leftTrigger2 = gamepad2.left_trigger;
            double rightTrigger2 = gamepad2.right_trigger;
            
            //claw
            if (xButtonPressed)
            {
                cs.clawMove(false);
            }
            else if (aButtonPressed)
            {
                cs.clawMove(true);
            }
            
            if (yButtonPressed && canShiftWristType)
            {
                canShiftWristType = false;
                if (normalWristType)
                {
                    cs.setWristMode('D');
                    normalWristType = false;
                }
                else
                {
                    cs.setWristMode('N');
                    normalWristType = true;
                }
            }
            else if (!yButtonPressed)
            {
                canShiftWristType = true;  
            }
            
            cs.update();
            
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
                dt.fieldOrientedTranslate(targetPowerX,targetPowerY,targetRotation, s1.GetImuReading());
            }
            else
            {
                dt.translate(targetPowerX,targetPowerY,targetRotation);
            }
            
            if (rightTrigger2 > 0.8)
            {
                armLiftInput = 1;
                armExtendInput = 1;
            }
            else if (leftTrigger2 > 0.8)
            {
                armLiftInput = -1;
                armExtendInput = -1;
            }
            
            am.rotate(armLiftInput, 'T');
            
            if (armExtendInput > 0.1)
            {
                e1.move(armExtendInput,600, 'T');
            }
            else if (armExtendInput < -0.1)
            {
                e1.move(armExtendInput,-125,'T');
            }
            else
            {
                e1.move(0,e1.GetCurrentPosition(),'T');
            }
            
            telemetry.addData("wrist setting",cs.GetWristState());
            telemetry.addData("wrist position", cs.GetWristPosition());
            telemetry.addData("angle double",am.GetNormalizedArmAngle());
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
        currentArmSpeed += speedInterval;
        am.SetSpeed(currentArmSpeed);
        canShiftArm = false;
    }
    
}

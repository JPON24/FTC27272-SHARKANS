package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmLiftMotor {
    public DcMotor armLiftL = null;
    public DcMotor armLiftR = null;
    private DcMotor extension = null;
    private double currentRotation = 0;
    
    double speed = 1;

    int topLimit = -6413;
    int bottomLimit = 0;
    
    public void init(HardwareMap hwMap)
    {
        armLiftL = hwMap.get(DcMotor.class, "armLiftL");
        armLiftL.setTargetPosition(0);
        armLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftL.setDirection(DcMotor.Direction.FORWARD);
        armLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        armLiftR = hwMap.get(DcMotor.class, "armLiftR");
        armLiftR.setTargetPosition(0);
        armLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftR.setDirection(DcMotor.Direction.FORWARD);
        armLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        extension = hwMap.get(DcMotor.class, "extension_1");
    }
    
    public void ResetEncoders()
    {
        topLimit = -6413;
        bottomLimit = 0;
        armLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    
    public void ResetEncodersUp()
    {
        ResetEncoders();
        topLimit = 0;
        bottomLimit = 6413;
    }
    
    public void SetSpeed(double change)
    {
        speed = change;
    }
   
    public double GetNormalizedArmAngle()
    {
        // 6413 = 260 * 24.6667
        double positionRange = 6413;
        double temp = 0;
        if (armLiftL.getCurrentPosition() != 0)
        {
            temp = Math.abs(armLiftL.getCurrentPosition()) / positionRange;
        }
        return temp;
    }
    public void rotate(double targetPower, char mode)
    {    
        int armPositionL = 0;
        int armPositionR = 0;
        if (mode == 'T')
        {
            if (targetPower > 0.1)
            {
                MoveToPositionTeleop(topLimit);
            }
            else if (targetPower < -0.1)
            {
                MoveToPositionTeleop(bottomLimit);
            }
            else
            {
                MoveToPositionTeleop(armLiftL.getCurrentPosition());
            }
        }
        else if (mode == 'A')
        {
        /*
        needed locations:
        0: 0
        5: -17
        30:-909
        45: -1212
        70: -1901
        90: -2497
        */

        // DEPRECATING -15 DEGREE SETTING, CAN ADD BACK LATER IF NEEDED
            int encoderPosition = ConvertAngleToEncoder((int)targetPower);
            MoveToPositionAutonomous(encoderPosition);
        }
    }
    
    private void MoveToPositionTeleop(int position)
    {
        armLiftL.setPower(speed);
        armLiftR.setPower(speed);
        
        armLiftL.setTargetPosition(position);
        armLiftR.setTargetPosition(position);
        
        armLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // currently untested and uncompiled, very well may error
    // might be better to use formula
    private int ConvertAngleToEncoder(int tgtPower)
    {
        return (int)(-tgtPower * 26.6667);
    }

    private void MoveToPositionAutonomous(int position)
    {
        armLiftL.setPower(speed);
        armLiftR.setPower(speed);
        
        armLiftL.setTargetPosition(position);
        armLiftR.setTargetPosition(position);
        
        armLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public int GetCurrentPosition()
    {
        return armLiftL.getCurrentPosition();
    }

    public boolean GetCompleted(int tgt)
    {
        // increased due to high range
        int lenience = 10;
        if (Math.abs(ConvertAngleToEncoder(tgt) - armLiftL.getCurrentPosition()) < lenience)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

  

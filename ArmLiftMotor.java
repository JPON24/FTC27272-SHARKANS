package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmLiftMotor {
    public DcMotor armLiftL = null;
    public DcMotor armLiftR = null;
    
    double speed = 0.4;

    int topLimit = -1513; // old value 1780
    int bottomLimit = 0;
    
    public void init(HardwareMap hwMap)
    {
        armLiftL = hwMap.get(DcMotor.class, "armLiftL");
        armLiftL.setTargetPosition(0);
        armLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        armLiftR = hwMap.get(DcMotor.class, "armLiftL");
        armLiftR.setTargetPosition(0);
        armLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        armLiftL.setDirection(DcMotor.Direction.REVERSE);
        armLiftR.setDirection(DcMotor.Direction.FORWARD);
    }
    
    public void ResetEncoders()
    {
        topLimit = -1513;
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
        bottomLimit = 1513;
    }
    
    public void SetSpeed(double change)
    {
        speed = change;
    }
   
    public double GetNormalizedArmAngle()
    {
        // 6933 = 260 * 6.846
        double positionRange = 1513;
        double temp = 0;
        if (armLiftL.getCurrentPosition() != 0)
        {
            temp = Math.abs(armLiftL.getCurrentPosition()) / positionRange;
        }
        return temp;
    }
    public void rotate(double targetPower, char mode)
    {    
        if (mode == 'T')
        {
            if (targetPower > 0.1)
            {
                MoveToPosition(topLimit);
            }
            else if (targetPower < -0.1)
            {
                MoveToPosition(bottomLimit);
            }
            else
            {
                MoveToPosition(armLiftL.getCurrentPosition() - 2);
            }
        }
        else if (mode == 'A')
        {
            int encoderPosition = ConvertAngleToEncoder((int)targetPower);
            MoveToPosition(encoderPosition);
        }
    }
    
    public void MoveToPosition(int position)
    {
        armLiftL.setPower(speed);
        armLiftR.setPower(speed);
        
        armLiftL.setTargetPosition(position);
        armLiftR.setTargetPosition(position);
    }

    public int ConvertAngleToEncoder(int tgtPower)
    {
        return (int)(-tgtPower * 6.846);
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

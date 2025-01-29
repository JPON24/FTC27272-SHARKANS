package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmLiftMotor {
    public DcMotor armLiftL = null;
    public DcMotor armLiftR = null;
    
    double speed = 0.8;

    int topLimit = -6933;
    int bottomLimit = 0;
    
    public void init(HardwareMap hwMap)
    {
        initArmMotor(armLiftL, hwMap, "armLiftL");
        initArmMotor(armLiftR, hwMap, "armLiftL");
    }

    //  code refactor
    private void initArmMotor(DcMotor motor,HardwareMap hwMap, String motorName)
    {
        motor = hwMap.get(DcMotor.class, motorName);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    public void ResetEncoders()
    {
        topLimit = -6933;
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
        bottomLimit = 6933;
    }
    
    public void SetSpeed(double change)
    {
        speed = change;
    }
   
    public double GetNormalizedArmAngle()
    {
        // 6933 = 260 * 26.6667
        double positionRange = 6933;
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
                MoveToPosition(armLiftL.getCurrentPosition());
            }
        }
        else if (mode == 'A')
        {
            int encoderPosition = ConvertAngleToEncoder((int)targetPower);
            MoveToPosition(encoderPosition);
        }
    }
    
    private void MoveToPosition(int position)
    {
        armLiftL.setPower(speed);
        armLiftR.setPower(speed);
        
        armLiftL.setTargetPosition(position);
        armLiftR.setTargetPosition(position);
    }

    private int ConvertAngleToEncoder(int tgtPower)
    {
        return (int)(-tgtPower * 26.6667);
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

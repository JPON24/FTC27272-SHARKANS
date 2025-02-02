package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

public class Extension_1 {
    public DcMotor extension_1 = null;
    
    
    public void init(HardwareMap hwMap)
    {
        extension_1 = hwMap.get(DcMotor.class, "extension_1");
        extension_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension_1.setTargetPosition(0);
        extension_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public void ResetEncoders()
    {
        extension_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void Move(double speed, int tgt, char mode)
    {
        if (mode == 'T')
        {
            if (speed > 0.1 && extension_1.getCurrentPosition() < 700)
            {
                MoveToPosition(700, speed);
            }
            else if (speed < -0.1 && extension_1.getCurrentPosition() > 0)
            {
                MoveToPosition(0, speed);
            }
            else
            {
                MoveToPosition(extension_1.getCurrentPosition(), 0);
            }
        }
        else if (mode == 'A')
        {
            MoveToPosition(tgt, speed);
        }
    }
    
    private void MoveToPosition(int tgtPosition, double speed)
    {
        extension_1.setTargetPosition(tgtPosition);
        extension_1.setPower(speed); 
    }

    public boolean GetCompleted(int tgt)
    {
        int lenience = 10;
        if (Math.abs(tgt - extension_1.getCurrentPosition()) < lenience)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    public int GetCurrentPosition()
    {
        return extension_1.getCurrentPosition();
    }
}

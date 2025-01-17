package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

public class Extension_1 {
    final double deadZone = 0.3;
    final double speed = 1;
    
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
    
    public void move(double targetPower, int tgt, String mode, int id)
    {
        if (mode == "T")
        {
            if (targetPower > deadZone && extension_1.getCurrentPosition() < 600)
            {
                extension_1.setTargetPosition(600);
                extension_1.setPower(speed * 0.9); 
            }
            else if(targetPower< -deadZone && extension_1.getCurrentPosition() > -125)
            {
                extension_1.setTargetPosition(-125);
                extension_1.setPower(-speed);
            }
            else
            {
                extension_1.setTargetPosition(extension_1.getCurrentPosition() + 25);
                extension_1.setPower(0);
            }
        }
        else if (mode == "A")
        {
            extension_1.setTargetPosition(tgt);
            if (targetPower > 0)
            {
                if (extension_1.getCurrentPosition() < tgt)
                {
                    extension_1.setPower(speed * 0.9);   
                }
                else
                {
                    
                    extension_1.setPower(0);   
                    extension_1.setTargetPosition(extension_1.getCurrentPosition() + 25);
                }
                    
            }
            else if (targetPower < 0)
            {
                if(extension_1.getCurrentPosition() > tgt)
                {
                    extension_1.setPower(-speed);
                }
                else
                {
                    
                    extension_1.setPower(0);
                    extension_1.setTargetPosition(extension_1.getCurrentPosition() + 25);
                }
            }
            else
            {
                extension_1.setPower(0);
            }
        }
    }

    public boolean GetCompleted(int tgt)
    {
        int lenience = 25;
        if (tgt - extension_1.getCurrentPosition() < Math.abs(lenience))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

public class Extension_1 {
    final double deadZone = 0.3;
    final double speed = 1;
    
    private DcMotor extension_1 = null;
    
    public void init(HardwareMap hwMap)
    {
        extension_1 = hwMap.get(DcMotor.class, "extension_1");
        extension_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //range: 0 to -508
    //min distance to rotate: -225
    public void move(double targetPower)
    {
        if (targetPower > deadZone)
        {
            extension_1.setPower(speed*0.4); 
        }
        else if(targetPower< -deadZone)
        {
            extension_1.setPower(-speed);
        }
        else
        {
            extension_1.setPower(0);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

public class Extension_1 {
    final double deadZone = 0.3; //amount of controller input needed to register input
    final double speed = 1; //speed at which it extends/retracts
    
    private DcMotor extension_1 = null;//reference to object

    //called from starter both and auton, makes reference to control hub to get dcmotor data and configures for use
    public void init(HardwareMap hwMap)
    {
        extension_1 = hwMap.get(DcMotor.class, "extension_1");
        extension_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//ensures that the position is intially 0
        extension_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//means that it will run solely off of set power, no target pos
    }
    //range: 0 to -508
    //min distance to rotate: -225
    public void move(double targetPower)
    {
        // if pushing up on stick
        if (targetPower > deadZone)
        {
            //extend the arm (lower speed)
            extension_1.setPower(speed*0.4); 
        }
        else if(targetPower< -deadZone) // if pushing down on stick
        {
            //retract the arm
            extension_1.setPower(-speed);
        }
        else //if not pushing stick
        {
            // do not move
            extension_1.setPower(0);
        }
    }
}

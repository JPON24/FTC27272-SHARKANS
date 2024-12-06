package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;



public class ClawServo{
    private Servo claw = null; // create servo variable to store servo data
    private double clawPos = 0.1; // current claw position

    //called in starter bot, intializes the claw by making reference to control hub
    public void init(HardwareMap hwMap)
    {
        claw = hwMap.get(Servo.class, "claw");
    }
    
    public void clawMove(boolean clawOpen)
    {
        // if the claw is open
        if (clawOpen){
            clawPos = 0.3; // set the claw's position to the open position
        }
        else { // if the claw is closed
            clawPos = 0.1; // set the claw's position to the closed position
        }
        
    }
    // runs all the time in teleop, handled in autonomous
    public void update() {
        claw.setPosition(clawPos); // repeatedly send power to the servo in order to make it grip instead of slipping
    }
}





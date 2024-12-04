package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;



public class ClawServo{
    private Servo claw = null;
    private double clawPos = 0.1;
    private double currentRotation;
    
    public void init(HardwareMap hwMap)
    {
        claw = hwMap.get(Servo.class, "claw");
    }
    
    public void clawMove(boolean clawOpen)
    {
        if (clawOpen){
            clawPos = 0.3;
        }
        else {
            clawPos = 0.1;
        }
        
    }
    
    public void update() {
        claw.setPosition(clawPos);
    }
}





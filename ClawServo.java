package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;

public class ClawServo{
    ArmLiftMotor am = new ArmLiftMotor();
    ElapsedTime runtime = new ElapsedTime();
    
    private Servo claw = null;
    private Servo wrist = null;
    
    private double clawPos = 0.1;
    private double wristPos = 0;
    private char wristMode = 'D';
    private double currentRotation;
    
    public void init(HardwareMap hwMap)
    {
        claw = hwMap.get(Servo.class, "claw");
        wrist = hwMap.get(Servo.class, "wrist");
        am.init(hwMap);
    }
    
    public void clawMove(boolean clawOpen)
    {
        if (clawOpen)
        {
            clawPos = 0.3; //initially
        }
        else
        {
            clawPos = 0.1;
        }
    }
    
    public void setWristMode(char temp)
    {
        wristMode = temp;
    }
    
    private void MoveWrist()
    {
        double tempWristPos = 0;
        if (wristMode == 'N') //normal
        {
            if (am.GetNormalizedArmAngle() < 0.573)
            {

                tempWristPos = 0.6 - (0.35 * am.GetNormalizedArmAngle());
            }
            else
            {
                // review and retest
                tempWristPos = 0 - (0.35 * am.GetNormalizedArmAngle());
            }
        }
        else if(wristMode == 'D') //down
        {
            tempWristPos = 0.6;
        }
        
        wristPos = tempWristPos;
    }
    
    public void update() {
        claw.setPosition(clawPos);
        MoveWrist();
        if (runtime.milliseconds() > 0.1)
        {
            wrist.setPosition(wristPos);
            runtime.reset();
        }
    }
    
    public char GetWristState()
    {
        return wristMode;
    }
    
    public boolean GetClawPosition()
    {
        if(clawPos == 0.3)
        {
            return true;
        }
        else 
        {
            return false;
        }
    }
}

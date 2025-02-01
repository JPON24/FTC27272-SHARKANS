package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        wrist.setDirection(Servo.Direction.FORWARD);
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
            clawPos = 0;
        }
    }
    
    public void setWristMode(char temp)
    {
        wristMode = temp;
    }

    // rom 230
    // gonna need some revision as angle range is not actually 221, a little different
    private void MoveToPosition()
    {
        double tempWristPos = 0;
        if (wristMode == 'N') //normal
        {
            if (am.GetNormalizedArmAngle() < 0.087) // less than 20 degrees
            {
                tempWristPos = 0.387 - am.GetNormalizedArmAngle();
            }
            else if (am.GetNormalizedArmAngle() < 0.478) // greater than 20 and less than 110
            {
                tempWristPos = 0.3 + 0.3 * ((am.GetNormalizedArmAngle() - 0.087)/0.391);
            }
            else if (am.GetNormalizedArmAngle() < 0.869) // greater than 110 and less than 200
            {
                tempWristPos = 0.3 * ((am.GetNormalizedArmAngle() - 0.478)/0.391);
            }
            else // greater than 200 and less than 230
            {
                tempWristPos = 0.3 + 0.1 * ((am.GetNormalizedArmAngle() - 0.87)/0.13);
            }
        }
        else if(wristMode == 'D') //down
        {
            tempWristPos = 0.05;
        }
        else if(wristMode == 'B') //back
        {
            tempWristPos = 0.6;
        }
        else
        {
            return;
        }
        
        wristPos = tempWristPos;
    }
    
    // used for shifting in custom mode
    public void MoveToPosition(double position)
    {
        wristPos = position;
    }
    
    public void Update() {
        MoveToPosition();
        if (runtime.seconds() > 0.125)
        {
            wrist.setPosition(wristPos);
            claw.setPosition(clawPos);
            runtime.reset();
        }
    }
    
    public char GetWristState()
    {
        return wristMode;
    }
    
    public double GetWristPosition()
    {
        return wrist.getPosition();
    }
    
    public boolean GetClawPosition()
    {
        // local claw pos
        if(clawPos == 0.3)
        {
            return true;
        }
        else 
        {
            return false;
        }
    }
    
    public double GetClawPositionReading()
    {
        return claw.getPosition();
    }
}

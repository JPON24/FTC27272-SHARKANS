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

    // gonna need some revision as angle range is not actually 221, a little different
    private void MoveWrist()
    {
        double tempWristPos = 0;
        if (wristMode == 'N') //normal
        {
            //tempWristPos = 0.6 * am.GetNormalizedArmAngle();
            if (am.GetNormalizedArmAngle() < 0.14) // less than 31 degrees
            {
                tempWristPos = 0.3 - (0.103 - 0.14 * am.GetNormalizedArmAngle());
            }
            else if (am.GetNormalizedArmAngle() < 0.573) // greater than 31 and less than 121
            {
                tempWristPos = 0.3 + 0.3 * ((am.GetNormalizedArmAngle() - 0.14)/0.433);
            }
            else // greater than 121 and less than 221
            {
                tempWristPos = 0.3 * ((am.GetNormalizedArmAngle() - 0.573)/0.427);
            }
        }
        else if(wristMode == 'D') //down
        {
            tempWristPos = 0;
        }
        
        wristPos = tempWristPos;
    }
    
    public void update() {
        claw.setPosition(clawPos);
        MoveWrist();
        if (runtime.seconds() > 0.5)
        {
            wrist.setPosition(wristPos);
            runtime.reset();
        }
    }
    
    public char GetWristState()
    {
        return wristMode;
    }
    
    public double GetWristPosition()
    {
        // return wrist.getPosition();
        return 0;
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

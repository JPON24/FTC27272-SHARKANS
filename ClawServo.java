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

    private CRServo wristR = null;
    private CRServo wristL = null;
//    private Servo wrist = null;

    private double clawPos = 0.0;
    private double wristPos = 0;
    double tempWristPos = 0;
    private char wristMode = 'D';
    private double currentRotation;
    
    public void init(HardwareMap hwMap, boolean auton)
    {
//        claw = hwMap.get(Servo.class, "claw");
//        wrist = hwMap.get(Servo.class, "wrist");
//        wrist.setDirection(Servo.Direction.FORWARD);
        wristR = hwMap.get(CRServo.class, "wristR");
        wristL = hwMap.get(CRServo.class, "wristL");
        am.init(hwMap, auton);
    }
    
    public void clawMove(boolean clawOpen)
    {
        if (clawOpen)
        {
            clawPos = 0.22; //initially
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
            tempWristPos = GetWristConstant('D');
        }
        else if(wristMode == 'B') //back
        {
            tempWristPos = GetWristConstant('B');
        }
        else if (wristMode == 'G')
        {
            tempWristPos = GetWristConstant('G');
        }
        else
        {
            return;
        }
        
        wristPos = tempWristPos;
    }

    private void MoveToPositionDiff()
    {
        double x = 0;
        double y = 0;
        if (wristMode == 'G')
        {
            x = 1;
            y = 0.65;
        }
        else if (wristMode == 'B')
        {
            x = 0;
            y = 0.39;
        }
        else
        {
            return;
        }
         MoveDiff(x,y);
    }
    
    // used for shifting in custom mode
    public void MoveToPosition(double position)
    {
        wristPos = position;
    }
    
    public void Update() {
//        MoveToPosition();
//        MoveToPositionDiff();
        if (runtime.seconds() > 0.125)
        {
//            wrist.setPosition(wristPos);
//            claw.setPosition(clawPos);
        }
//        wristL.setPower((stickY - stickX)/2);
//        wristR.setPower((stickY + stickX)/2);
    }
    
    public void ResetRuntime()
    {
        runtime.reset();
    }
    
    public char GetWristState()
    {
        return wristMode;
    }
    
    public double GetWristPosition()
    {
//        return wrist.getPosition();
        return 0;
    }

    public void MoveDiff(double x, double y)
    {
        double outputL = y-x;
        double outputR = y+x;
        if (Math.abs(x) + Math.abs(y) > 1)
        {
            outputL /= 2;
            outputR /= 2;
        }
        wristL.setPower(outputL);
        wristR.setPower(outputR);
    }

    public double GetWristConstant(char mode)
    {
        switch (mode)
        {
            case 'D':
                return 0.05;
            case 'B':
                return 0.65; // old 0.585
            case 'G':
                return 0.39; // old 0.33  
            default:
                return 0;
        }
    }
    
    public boolean GetClawClosed()
    {
        // local claw pos
        if(claw.getPosition() == 0.22)
        {
            return true;
        }
        else 
        {
            return false;
        }
    }
    
    public double GetClawPosition()
    {
        return claw.getPosition();
    }
}

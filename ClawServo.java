package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;

public class ClawServo{
    ArmLiftMotor am = new ArmLiftMotor();
    ElapsedTime runtime = new ElapsedTime();
    
    private Servo claw = null;

    private Servo wristR = null;
    private Servo wristL = null;

    private double clawPos = 0.0;

    private double wristPosL = 0;
    private double wristPosR = 0;

    private char wristMode = 'D';
    private double currentRotation;

    private boolean auton = true;
    
    public void init(HardwareMap hwMap, boolean auton)
    {
        wristR = hwMap.get(Servo.class, "wristR");
        wristL = hwMap.get(Servo.class, "wristL");
        wristL.setDirection(Servo.Direction.FORWARD);
        wristR.setDirection(Servo.Direction.REVERSE);
        am.init(hwMap, auton);
        this.auton = auton;
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
    
    private void MoveToPositionDiff()
    {
        // 0.5 is normal position

        if (wristMode == 'G')
        {
            wristPosL = 0.31;
            wristPosR = 0.31;
        }
        else if (wristMode == 'B')
        {
            wristPosL = 0;
            wristPosR = 0.58;
        }
        else
        {
            wristPosL = 0;
            wristPosR = 0;
        }
    }

    public void Update() {
        if (runtime.seconds() > 0.125)
        {
            if (auton)
            {
                SetDiffPos(wristPosL,wristPosR);
            }
//            claw.setPosition(clawPos);
        }
    }
    
    public void ResetRuntime()
    {
        runtime.reset();
    }

    public void SetWristMode(char temp)
    {
        wristMode = temp;
        MoveToPositionDiff();
    }

    public void MoveDiff(double x, double y)
    {
        wristL.setPosition(1 - ((y + x) / 2));
        wristR.setPosition((y + x) / 2);
    }

    public void SetDiffPos(double positionL, double positionR)
    {
        wristL.setPosition(positionL);
        wristR.setPosition(positionR);
    }
    
    public boolean GetClawClosed()
    {
        return claw.getPosition() == 0.22;
    }
    
    public double GetClawPosition()
    {
        return claw.getPosition();
    }
    
    public char GetWristState()
    {
        return wristMode;
    }
    
    public double GetWristLPosition()
    {
        return wristL.getPosition();
    }

    public double GetWristRPosition()
    {
        return wristR.getPosition();
    }

    public double GetWristLTgt(double x, double y)
    {
        double output = y-x;
        if (Math.abs(x) + Math.abs(y) > 1)
        {
            output /= 2;
        }
        return output;
    }

    public double GetWristRTgt(double x, double y)
    {
        double output = y+x;
        if (Math.abs(x) + Math.abs(y) > 1)
        {
            output /= 2;
        }
        return output;
    }
}

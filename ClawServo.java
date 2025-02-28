package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    private boolean auton = true;
    
    public void init(HardwareMap hwMap, boolean auton)
    {
        wristR = hwMap.get(Servo.class, "wristR");
        wristL = hwMap.get(Servo.class, "wristL");
        claw = hwMap.get(Servo.class, "claw");
        am.init(hwMap);
        this.auton = auton;
    }
    
    public void SetClawOpen(boolean temp)
    {
        if (temp)
        {
            clawPos = 0.22; //initially
        }
        else
        {
            clawPos = 0;
        }
    }

    // rezero
    private void MoveToPositionDiff()
    {
        // 0.5 is normal position

        if (wristMode == 'G')
        {
            wristPosL = 0;
            wristPosR = 0;
        }
        else if (wristMode == 'B')
        {
            wristPosL = 1;
            wristPosR = 0.555;
        }
        else if (wristMode == 'M')
        {
            wristPosL = 0.55;
            wristPosR = 0.55;
        }
        else if (wristMode == 'D')
        {
            wristPosL = 0.2;
            wristPosR = 0.2;
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
            claw.setPosition(clawPos);
        }
    }

    public void SetWristMode(char temp)
    {
        wristMode = temp;
        MoveToPositionDiff();
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
}

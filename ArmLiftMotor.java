package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class ArmLiftMotor {
    public DcMotor armLift = null;
    private DcMotor extension = null;
    private double currentRotation = 0;
        
    double rpm = 84;
    double gearRatio = 5;
    
    double countsPerGearRev = rpm * gearRatio;
    double countsPerDegree = countsPerGearRev/360;

    double speed = 1;

    int topLimit = -2500;
    int bottomLimit = 300;
    
    public void init(HardwareMap hwMap)
    {
        armLift = hwMap.get(DcMotor.class, "armLiftMotor");
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        extension = hwMap.get(DcMotor.class, "extension_1");
    }
    
    public void ResetEncoders()
    {
        topLimit = -2500;
        bottomLimit = 300;
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ResetEncodersUp()
    {
        ResetEncoders();
        topLimit = 0;
        bottomLimit = 2800;
    }
    
    public void SetSpeed(double change)
    {
        speed = change;
    }
   
    public double GetNormalizedArmAngle()
    {
        double positionRange = Math.abs(topLimit) + Math.abs(bottomLimit);
        double temp = 0;
        if (armLift.getCurrentPosition() != 0)
        {
            temp = Math.abs(armLift.getCurrentPosition()) / positionRange;
        }
        return temp;
    }
   
    public void rotate(double targetPower, String mode,int id)
    {    
        int armPosition = 0;
        if (mode == "T")
        {
            if (targetPower > 0.1)
            {
                armLift.setTargetPosition(topLimit);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setPower(speed);
            }
            else if (targetPower < -0.1)
            {
                armLift.setTargetPosition(bottomLimit);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setPower(speed);
            }
            else
            {
                armLift.setTargetPosition(armLift.getCurrentPosition() + ((int)countsPerDegree * 1));
                armLift.setPower(speed);
            }
        }
        else if (mode == "A")
        {
        /*
        needed locations:
        0: 0
        5: -17
        30:-909
        45: -1212
        70: -1901
        90: -2497
        */

        // DEPRECATING -15 DEGREE SETTING, CAN ADD BACK LATER IF NEEDED
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int encoderPosition = ConvertAngleToEncoder(targetPower);
            MoveToPositionAutonomous(encoderPosition);
        }
    }

    // currently untested and uncompiled, very well may error
    // might be better to use formula
    private int ConvertAngleToEncoder(int tgtPower)
    {
        switch ((int)targetPower){
                case 5:
                    return -17;
                case 30:
                    return -909;
                case 45:
                    return -1212;
                case 50:
                    return -1550;
                case 60:
                    return -1700;
                case 70:
                    return -1901;
                case 80:
                    return -2000;
                case 90:
                    return -2200;
            }
    }

    private MoveToPositionAutonomous(int position)
    {
        armLift.setTargetPosition(position);
        if (armLift.getCurrentPosition()>position){
            armLift.setPower(-speed);
        }
        else
        {
            armLift.setPower(speed);
        }
    }

    public boolean GetCompleted(int tgt)
    {
        // increased due to high range
        int lenience = 50;
        if (ConvertAngleToEncoder(tgt) - armLift.getCurrentPosition() < Math.abs(lenience))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

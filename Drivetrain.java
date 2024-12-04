package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Drivetrain{
    final int mmFtConversion = 305; 
    final float vectorPadding = 0.2F;
    
    double rpm = 312;
    double gearRatio = 1;
    double gearReduction = 19.1;
    double wheelCircumference = 76.2 * 3.14;
    double countsPerGearRev = rpm * gearRatio;
    double countsPerDegree = countsPerGearRev/360;
    double countsPerWheelRev = countsPerGearRev * gearReduction;
    double countsPerMM = countsPerWheelRev/wheelCircumference;
    
    static double initialPosition;
    static boolean calibrating = false;
    static public int actionId = 0;
    
    private double speedScalar = 1;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    
    public void init(HardwareMap hwMap)
    {
        frontLeft = hwMap.get(DcMotor.class, "FrontLeft");
        frontRight = hwMap.get(DcMotor.class, "FrontRight");
        backLeft = hwMap.get(DcMotor.class, "BackLeft");
        backRight = hwMap.get(DcMotor.class, "BackRight");
        
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void Calibrate(double targetPosition)
    {
        calibrating = true;
        initialPosition = frontLeft.getCurrentPosition();
        frontLeft.setTargetPosition((int)(targetPosition * countsPerMM));
        backLeft.setTargetPosition((int)(targetPosition * countsPerMM));
        frontRight.setTargetPosition((int)(targetPosition * countsPerMM));
        backRight.setTargetPosition((int)(targetPosition * countsPerMM));
    }
        
    public void targetPositionTranslate(int targetPowerX, int targetPowerY, int rotation, double targetPosition, int id)
    {
        // left: positive
        // down: positive
        // right: negative
        // up: negative
        if(!calibrating)
        {
            Calibrate(targetPosition);
        }
        if(id!=actionId) {return;}
        switch(targetPowerX)
        {
            case 1:
                if (frontLeft.getCurrentPosition() < initialPosition - targetPosition * countsPerMM)
                {
                    translate(1,0,0);
                }
                else
                {
                    translate(0,0,0);
                    calibrating = false;
                    actionId++;
                }
                break;
            case -1:
                if (frontLeft.getCurrentPosition() > initialPosition + targetPosition * countsPerMM)
                {
                    translate(-1,0,0);
                }
                else
                {
                    translate(0,0,0);
                    calibrating = false;
                    actionId++;
                }
                break;
            default:
                break;
        }    
        switch(targetPowerY)
        {
            case 1:
                if (frontLeft.getCurrentPosition() < initialPosition - targetPosition * countsPerMM)
                {
                    translate(0,-1,0);
                }
                else
                {
                    translate(0,0,0);
                    calibrating = false;
                    actionId++;
                }
                break;
            case -1:
                if (frontLeft.getCurrentPosition() > initialPosition + targetPosition * countsPerMM)
                {
                    translate(0,1,0);
                }
                else
                {
                    translate(0,0,0);
                    calibrating = false;
                    actionId++;
                }
               break;
            default:
               break;
        }
        switch(rotation)
        {
            case 1:
                if (frontLeft.getCurrentPosition() - countsPerDegree < initialPosition - targetPosition * countsPerDegree)
                {
                    translate(0,0,1);
                }
                else
                {
                    translate(0,0,0);
                    calibrating = false;
                    actionId++;
                }
                break;
            case -1:
                if (frontLeft.getCurrentPosition() + countsPerDegree > initialPosition + targetPosition * countsPerDegree)
                {
                    translate(0,0,-1);
                }
                else
                {
                    translate(0,0,0);
                    calibrating = false;
                    actionId++;
                }
                break;
            default:
                break;
        }
        /*
        double frontLeftPower = -rotation + targetPowerY -targetPowerX;
        double frontRightPower = -rotation -targetPowerY -targetPowerX;
        double backLeftPower = -rotation + targetPowerY + targetPowerX;
        double backRightPower = -rotation -targetPowerY + targetPowerX;
        
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);*/
    }

    
    public void translate(double targetPowerX, double targetPowerY, double rotation)
    {
        /*frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);*/
        
        double frontLeftPower = -rotation + targetPowerY -targetPowerX;
        double frontRightPower = -rotation -targetPowerY -targetPowerX;
        double backLeftPower = -rotation + targetPowerY + targetPowerX;
        double backRightPower = -rotation -targetPowerY + targetPowerX;
        
        frontLeft.setPower(frontLeftPower * speedScalar);
        frontRight.setPower(frontRightPower * speedScalar);
        backLeft.setPower(backLeftPower * speedScalar);
        backRight.setPower(backRightPower * speedScalar);
    }
    
    public void SetSpeedScalar(double change)
    {
        speedScalar = change;
    }
}

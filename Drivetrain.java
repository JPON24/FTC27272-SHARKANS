//package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain{
    private double speedScalar = 1; // used to control speed with bumpers
    
    //motor references
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    IMU imu;
    
    public void init(HardwareMap hwMap)
    {
        // intialize motors
        frontLeft = hwMap.get(DcMotor.class, "FrontLeft");
        frontRight = hwMap.get(DcMotor.class, "FrontRight");
        backLeft = hwMap.get(DcMotor.class, "BackLeft");
        backRight = hwMap.get(DcMotor.class, "BackRight");

        //intialize imu
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot robotOrientationInit = new RevHubOrientationOnRobot(logoDir,usbDir);
        imu.initialize(new IMU.Parameters(robotOrientationInit));
        imu.resetYaw();

        //set their run modes to run without encoder, meaning they will not be using setTargetPos
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void fieldOrientedTranslate(double targetPowerX, double targetPowerY, double rotation)
    {
        // get yaw
        Orientation o;
        o = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        double yaw = o.firstAngle;
        
        double stickRotation = 0;
        int sameSignFlip = 1;
        if (targetPowerY > 0 && targetPowerX < 0) //quad2
        {
            sameSignFlip = -1;
            stickRotation = (Math.atan2(targetPowerY,targetPowerX) - Math.PI) * 180/Math.PI;   
        }
        else if (targetPowerY < 0 && targetPowerX < 0) //quad3
        {
            sameSignFlip = -1;
            stickRotation = (Math.atan2(targetPowerY,targetPowerX) + Math.PI) * 180/Math.PI;
        }
        else //quad1 and quad4
        {
            sameSignFlip = 1;
            stickRotation = Math.atan2(targetPowerY,targetPowerX) * 180/Math.PI;
        }
        
        // angle of imu yaw supplemented by the stick's rotation, determined by atan
        double theta = (360-yaw) + stickRotation;
        double power = Math.hypot(targetPowerX,targetPowerY); //get hypotenuse of x and y tgt, getting the power
        
        double speedScalarLocal = 0.5; // static speed scalar, can be adjusted as seen fit
        
        
        // if at max power diag, limit to magnitude of 1
        // with the normalizing code, the diag movement had a bug where max power (being magnitude sqrt(2))-- 
        // --would cause wheels to flip polarity
        // to counteract this the power is limited to a proper magnitude
        if (power > 1)
        {
            power = 1;
        }
        else if (power < -1) 
        {
            power = -1;
        }
        
        //get the sin and cos of theta
        //math.pi/4 represents 45 degrees, accounting for angular offset of mechanum
        double sin = Math.sin((theta * (Math.PI/180)) - (Math.PI/4));
        double cos = Math.cos((theta * (Math.PI/180)) - (Math.PI/4));
        //max of sin and cos, used to normalize the values for maximum efficiency
        double maxSinCos = Math.max(Math.abs(sin),Math.abs(cos));
    
        //same sign flip is to account for the inability of atan2, it typically only works for quadrants 1 and 4
        //by flipping the polarity when x < 0, we can use atan for every quadrant
        double flPower = (power*cos/maxSinCos+rotation) * sameSignFlip;
        double frPower = (power*sin/maxSinCos-rotation) * sameSignFlip;
        double blPower = (power*sin/maxSinCos+rotation) * sameSignFlip;
        double brPower = (power*cos/maxSinCos-rotation) * sameSignFlip;
        
        //another normalization
        if ((power + Math.abs(rotation)) > 1)
        {
            flPower /= power + rotation;
            frPower /= -power - rotation;
            blPower /= power + rotation;
            brPower /= -power - rotation;
        }
        
        //set speed to calculated values * the speedScalar(determined by bumpers)
        frontLeft.setPower(flPower * speedScalar);
        frontRight.setPower(frPower * speedScalar);
        backLeft.setPower(blPower * speedScalar);
        backRight.setPower(brPower * speedScalar);
    }
    
    public void translate(double targetPowerX, double targetPowerY, double rotation)
    {
        // inputs target power on x and y, outputs proper power distribution
        
        double theta = Math.atan2(targetPowerY, targetPowerX);
        double power = Math.hypot(targetPowerX, targetPowerY);
        
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));
        
        double flPower = power * cos/max + rotation;
        double frPower = power * sin/max - rotation;
        double blPower = power * sin/max + rotation;
        double brPower = power * cos/max - rotation;

        if ((power + Math.abs(rotation)) > 1)
        {
            flPower /=power + rotation;
            frPower /=power + rotation;
            blPower /=power + rotation;
            brPower /=power + rotation;
        }
        
        //sets power of each motor
        frontLeft.setPower(flPower * speedScalar);
        frontRight.setPower(frPower * speedScalar);
        backLeft.setPower(blPower * speedScalar);
        backRight.setPower(brPower * speedScalar);
    }
    
    public void SetSpeedScalar(double change)
    {
        //sets the speed scalar of the motors to a specified value in starter bot
        speedScalar = change;
    }
}

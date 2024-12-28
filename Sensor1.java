package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.Drivetrain;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.util.ElapsedTime;


public class Sensor1 {
    Drivetrain dt = new Drivetrain();
    ActionHandler action = new ActionHandler();
    //ElapsedTime runtime = new ElapsedTime();
    
    IMU imu;
    MaxSonarI2CXL leftSensor = null;
    MaxSonarI2CXL backSensor = null;
    MaxSonarI2CXL rightSensor = null;

    SparkFunOTOS odometry;
    SparkFunOTOS.Pose2D pos;
    
    boolean calibrating = false;
    
    double initialDistanceL = 0;
    double initialDistanceB = 0;
    double initialDistanceR = 0;
    
    public void init(HardwareMap hwMap)
    {
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot robotOrientationInit = new RevHubOrientationOnRobot(logoDir,usbDir);
        imu.initialize(new IMU.Parameters(robotOrientationInit));
        imu.resetYaw();

        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        leftSensor = hwMap.get(MaxSonarI2CXL.class, "Sensor1");
        backSensor = hwMap.get(MaxSonarI2CXL.class, "Sensor2");
        rightSensor = hwMap.get(MaxSonarI2CXL.class, "Sensor3");
        odometry.begin();
        //odometry.calibrateIMU();
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        dt.init(hwMap);
    }
    
    public void OdometryControl(double speed, double tgtX,double tgtY,double tgtRot)
    {
        if (!odometry.isConnected()) {return;}

        // Orientation o;
        // o = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        // double yaw = o.firstAngle;
        // yaw = 360-yaw;
        double yaw = GetImuReading();
        double rot = 0;
        double lenience = 3;

        pos = odometry.getPosition();
        
        SparkFunOTOS.Pose2D tgtPos = new SparkFunOTOS.Pose2D();
        tgtPos.x = tgtX;
        tgtPos.y = tgtY;

        SparkFunOTOS.Pose2D moveVector = new SparkFunOTOS.Pose2D();
        
        double xDiff = tgtPos.x - pos.x;
        double yDiff = tgtPos.y - pos.y;
        double maxXYDiff = Math.max(Math.abs(xDiff), Math.abs(yDiff));
        xDiff/=maxXYDiff;
        yDiff/=maxXYDiff;
        
        moveVector.x = xDiff;
        moveVector.y = yDiff;
        
        if (yaw < tgtRot - lenience)
        {
            rot = speed;
        }
        else if (yaw > tgtRot + lenience)
        {
            rot = -speed;
        }
        
        /*if (tgtPos.x - pos.x > 0)
        {
            moveVector.x = speed;
        }
        else if (tgtPos.x - pos.x < 0)
        {
            moveVector.x = -speed;
        }
        else
        {
            moveVector.x = 0;
        }

        if (tgtPos.y - pos.y > 0)
        {
            moveVector.y = speed;
        }
        else if (tgtPos.y - pos.y < 0)
        {
            moveVector.y = -speed;
        }
        else
        {
            moveVector.y = 0;
        }

        if (yaw < tgtRot - lenience)
        {
            rot = speed;
        }
        else if (yaw > tgtRot + lenience)
        {
            rot = -speed;
        }*/

        dt.fieldOrientedTranslate(moveVector.x,moveVector.y,rot);
    }

    public double GetImuReading()
    {
        return odometry.getPosition().h;
    }
    
    public double GetPositionX()
    {
        return odometry.getPosition().x;
    }
    
    public double GetPositionY()
    {
        return odometry.getPosition().y;
    }

    public void Calibrate()
    {
        calibrating = true;
        initialDistanceL = leftSensor.getDistanceSync(DistanceUnit.INCH);
        initialDistanceB = backSensor.getDistanceSync(DistanceUnit.INCH);
        initialDistanceR = rightSensor.getDistanceSync(DistanceUnit.INCH);
    }
    
    public void moveToPositionR(double targetPosition,double speed, char dir,int id)
    {
        if(!calibrating)
        {
            Calibrate();
        }
        double currentDistanceR = rightSensor.getDistanceSync(DistanceUnit.INCH);
        double currentDistanceB = backSensor.getDistanceSync(DistanceUnit.INCH);
        if(dir == 'F' || dir == 'B')//forward and back
        {
            switch(dir)
            {
                case 'F':
                    if(currentDistanceB < initialDistanceB + targetPosition)
                    {
                        //currentDistanceB = backSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(0,speed,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
                case 'B':
                    if (currentDistanceB > initialDistanceB - targetPosition)
                    {
                        //currentDistanceB = backSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(0,-speed,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
            }
        }
        else
        {
            switch(dir)
            {
                case 'L':
                    if (currentDistanceR > initialDistanceR - targetPosition)
                    {
                        //currentDistanceS = sideSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(-speed,0,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
                case 'R':
                    if (currentDistanceR < initialDistanceR + targetPosition)
                    {
                        //currentDistanceS = sideSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(speed,0,0); 
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
            }
        }
    }
    
    public void moveToPositionL(double targetPosition,double speed, char dir,int id)
    {
        if(!calibrating)
        {
            Calibrate();
        }
        double currentDistanceL = leftSensor.getDistanceSync(DistanceUnit.INCH);
        double currentDistanceB = backSensor.getDistanceSync(DistanceUnit.INCH);
        if(dir == 'F' || dir == 'B')//forward and back
        {
            switch(dir)
            {
                case 'F':
                    if(currentDistanceB < initialDistanceB + targetPosition)
                    {
                        //currentDistanceB = backSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(0,-speed,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        calibrating = false;
                        action.IncrementActionId();
                    }
                    break;
                case 'B':
                    if (currentDistanceB > initialDistanceB - targetPosition)
                    {
                        //currentDistanceB = backSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(0,speed,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        calibrating = false;
                        action.IncrementActionId();
                    }
                    break;
            }
        }
        else
        {
            switch(dir)
            {
                case 'L':
                    if (currentDistanceL > initialDistanceL - targetPosition)
                    {
                        //currentDistanceS = sideSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(speed,0,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        calibrating = false;
                        action.IncrementActionId();
                    }
                    break;
                case 'R':
                    if (currentDistanceL < initialDistanceL + targetPosition)
                    {
                        //currentDistanceS = sideSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(-speed,0,0); 
                    }
                    else
                    {
                        calibrating = false;
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
            }
        }
    }
}

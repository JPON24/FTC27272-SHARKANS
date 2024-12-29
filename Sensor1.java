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

    boolean[] completedBools = new boolean[3];
    
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
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        dt.init(hwMap);
    }
    
    public void OdometryControl(double speed, double tgtX,double tgtY,double tgtRot)
    {
        if (!odometry.isConnected()) {return;}

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
        
        if (xDiff < 1)
        {
            completedBools[0] = true;
        }
        else
        {
            completedBools[0] = false;
        }

        if (yDiff < 1)
        {
            completedBools[1] = true;
        }
        else
        {
            completedBools[1] = false;
        }

        double maxXYDiff = Math.max(Math.abs(xDiff), Math.abs(yDiff));
        xDiff/=maxXYDiff;
        yDiff/=maxXYDiff;
        
        moveVector.x = xDiff;
        moveVector.y = yDiff;
        
        if (yaw < tgtRot - lenience)
        {
            rot = speed;
            completedBools[2] = true;
        }
        else if (yaw > tgtRot + lenience)
        {
            rot = -speed;
            completedBools[2] = false;
        }
        else
        {
            completedBools[2] = true;
        }

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

    public bool AllBoolsCompleted()
    {
        for (int i = 0; i < 3; i++)
        {
            if (completedBools[i] == false)
            {
                return false;
            }
            else
            {
                continue;
            }
        }
        return true;
    }
}   

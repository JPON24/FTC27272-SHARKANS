package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;


public class Sensor1 {
    Drivetrain dt = new Drivetrain();
    ActionHandler action = new ActionHandler();
    ElapsedTime runtime = new ElapsedTime();

    SparkFunOTOS odometry;
    SparkFunOTOS.Pose2D pos;

    boolean[] completedBools = new boolean[3];
    boolean[] canStartTimers = new boolean[3];
    double[] initTimes = new double[3];
    int[] polarities = new int[3];

    public void init(HardwareMap hwMap)
    {
        for (int i = 0; i < 3; i++) {canStartTimers[i] = true;}
        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        odometry.begin();
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        dt.init(hwMap);
    }
    
    public void OdometryControl(double speed, double tgtX,double tgtY,double tgtRot)
    {
        if (!odometry.isConnected()) {return;}

        double yaw = GetImuReading();
        double lenience = 3;

        pos = odometry.getPosition();
        
        SparkFunOTOS.Pose2D tgtPos = new SparkFunOTOS.Pose2D();
        tgtPos.x = tgtX;
        tgtPos.y = tgtY;
        tgtPos.h = tgtRot;

        SparkFunOTOS.Pose2D moveVector = new SparkFunOTOS.Pose2D();
        
        double xDiff = tgtPos.x - pos.x;
        double yDiff = tgtPos.y - pos.y;
        double rotDiff = tgtPos.y - pos.h;

        if (Math.abs(xDiff) < 1)
        {
            completedBools[0] = true;
            SetInitTime(0);
            PolaritySwap(0);
        }
        else
        {
            completedBools[0] = false;
            canStartTimers[0] = true;
        }

        if (Math.abs(yDiff) < 1)
        {
            completedBools[1] = true;
            SetInitTime(1);
            PolaritySwap(1);
        }
        else
        {
            completedBools[1] = false;
            canStartTimers[1] = true;
        }

        double maxXYDiff = Math.max(Math.abs(xDiff), Math.abs(yDiff));
        xDiff/=maxXYDiff;
        yDiff/=maxXYDiff;
        
        if (pos.h - lenience < rotDiff)
        {
            moveVector.h = speed;
            completedBools[2] = false;
            canStartTimers[2] = true;
        }
        else if (pos.h + lenience > rotDiff)
        {
            moveVector.h = -speed;
            completedBools[2] = false;
            canStartTimers[2] = true;
        }
        else
        {
            completedBools[2] = true;
            SetInitTime(2);
            PolaritySwap(2);
        }        
        
        moveVector.x = xDiff * polarities[0];
        moveVector.y = yDiff * polarities[1];
        moveVector.h *= polarities[2];

        dt.fieldOrientedTranslate(moveVector.x,moveVector.y,moveVector.h);
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

    private void SetInitTime(int index)
    {
        if (canStartTimers[index])
        {
            initTimes[index] = runtime.milliseconds();
            canStartTimers[index] = false;
        }
    }

    private void PolaritySwap(int index)
    {
        if (runtime.milliseconds() - initTimes[index] > 0.05)
        {
            polarities[index] = 1;
        }
        else
        {
            polarities[index] = -1;
        }
    }
}   

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.util.ElapsedTime;


public class OdometrySensor {
    Drivetrain dt = new Drivetrain();
    ElapsedTime runtime = new ElapsedTime();
    SparkFunOTOS odometry; 
    SparkFunOTOS.Pose2D pos;

    boolean[] completedBools = new boolean[3];
    
    double deltaTime, last_time;
    double integral, previous = 0;
    double kp, ki, kd;
    double[] setpoint = new double[3];
    double[] output = new double[3];
    
    double pid(double error)
    {
        double proportional = error;
        integral += error * deltaTime;
        double derivative = (error - previous) / deltaTime;
        previous = error;
        double output = (kp * proportional) + (ki * integral) + (kd * derivative);
        return output;
    }
    
    //y
    //offset from front - 12 3/8
    //offset from back - 4 7/8
    //length of robot - 17.25
    
    //x
    //offset from left side 7 7/16
    // 7 13/16 offset
    // 7 10/16 center
    // offset = 3/16
    //width of robot - 15.25
    
    public void init(HardwareMap hwMap)
    {
        kp = 0.0;
        ki = 0.2;
        kd = 0.001;
        last_time = 0;
        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        odometry.resetTracking();
        odometry.begin();
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.setLinearScalar(40/41);
        odometry.setAngularScalar(1);
        odometry.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte)0x0D));// disables accelerometer
        odometry.setOffset(new SparkFunOTOS.Pose2D(0.1875,-3.75,0));
        dt.init(hwMap);
    }
    
    public void OdometryControl(double speed, double tgtX,double tgtY,double tgtRot)
    {
        if (!odometry.isConnected()) {return;}

        double yaw = GetImuReading();
        double rot = 0;
        double distanceLenience = 2;
        double angleLenience = 10;
        
        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;
        
        pos = odometry.getPosition();
        setpoint[0] = tgtX;
        setpoint[1] = tgtY;
        setpoint[2] = rot;
        
        double[] errors = new double[3];
        for (int i = 0; i < 3; i++)
        {
            output[i] = pid(errors[i]);
        }
        
        // SparkFunOTOS.Pose2D tgtPos = new SparkFunOTOS.Pose2D();
        // tgtPos.x = tgtX;
        // tgtPos.y = tgtY;

        // SparkFunOTOS.Pose2D moveVector = new SparkFunOTOS.Pose2D();
        // double xDiff = tgtPos.x - pos.x;
        // double yDiff = tgtPos.y - pos.y;
        
        // if (Math.abs(xDiff) < distanceLenience)
        // {
        //     completedBools[0] = true;
        // }
        // else
        // {
        //     completedBools[0] = false;
        // }
        
        if (Math.abs(output[0]) < distanceLenience)
        {
            completedBools[0] = true;
        }
        else
        {
            completedBools[0] = false;
        }

        if (Math.abs(output[1]) < distanceLenience)
        {
            completedBools[1] = true;
        }
        else
        {
            completedBools[1] = false;
        }

        // double maxXYDiff = Math.max(Math.abs(xDiff), Math.abs(yDiff));
        // xDiff/=maxXYDiff;
        // yDiff/=maxXYDiff;
        double maxXYOutput = Math.max(Math.abs(output[0]),Math.abs(output[1]));
        output[0] /= maxXYOutput;
        output[1] /= maxXYOutput;
        
        // moveVector.x = xDiff;
        // moveVector.y = yDiff;
        
        if (Math.abs(output[2]) < angleLenience)
        {
            completedBools[2] = false;
        }
        else
        {
            completedBools[2] = true;
        }

        dt.fieldOrientedTranslate(output[0] * speed, output[1] * speed,output[2] * speed);
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

    public boolean AllBoolsCompleted()
    {
        for (int i = 0; i < 3; i++)
        {
            if (completedBools[i] == false)
            {
                return false;
            }
        }
        return true;
    }
    
    public void OverwriteAllBoolsCompleted(boolean state)
    {
        for (int i = 0; i < 3; i++)
        {
            completedBools[i] = state;
        }
    }
}   

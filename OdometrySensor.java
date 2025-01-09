package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class OdometrySensor {
    Drivetrain dt = new Drivetrain();
    ElapsedTime runtime = new ElapsedTime();
    SparkFunOTOS odometry; 
    SparkFunOTOS.Pose2D pos;

    boolean[] completedBools = new boolean[3];
    
    double deltaTime, last_time;
    double integral, previous = 0;
    double kp, ki, kd;
    double[] output = new double[3];
    double[] errors = new double[3];
    
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
    
    /*  <//PID TESTING LOG//>
    1,0,0: 2/-2
    2,0,0: SPIN TO WIN
    1.41,0,0: 0.5/-0.5
    1.41,0.1,0: spinning weirdly (turns out it had picked up a spikemark)
    1.5,0,0: better y and x error, h error worse
    1.6,0,0: better error: -0.4/4 pos -14/14 rot
    2,0,0: error worse: -0.6/0.6 pos -16/16 rot
    1.8,0,0: -0.5/0.5 pos -6/6 rot
    1.75,0,0: pos better rot worse
    1.8,1,0: rotated quickly into wall
    1.8,1,1: same as 1.41,0.1,0
    1.8,0,1: pos slightly lower, rot between -3/-11
    
    1.8,0.15,0.1: 
    1.8,0.15,0.18 kinda good
    
    best currently: 0.9,0.2,0.09
    current best: 0.9,0.15,0.1
    current best: 0.9,0.2,0.2
    
    0.45,0.1,0.1 good
    0.45 0.045 0
    
    0.5/-0.5 with most of the time between magnitude of 0.2
    
    1.55 0.32 0 = absolute holy grail :D
    */
    
    
    public void init(HardwareMap hwMap)
    {
        //currently using pid on odometry position readings, could be causing some of the oscillation problems
        // might be better to use motor ticks with our odometry for more accuracy
        kp = 1.55;
        ki = 0.313; 
        kd = 0.155; 
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
    
    double pid(double error)
    {
        integral += error * runtime.seconds();
        double derivative = (error - previous) / runtime.seconds();
        previous = error;
        double output = (kp * error) + (ki * integral) + (kd * derivative);
        runtime.reset();
        return output;
    }
    
    public void OdometryControl(double speed, double tgtX,double tgtY,double tgtRot)
    {
        if (!odometry.isConnected()) {return;}

        double distanceLenience = 2;
        double angleLenience = 3;
        
        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;
        
        pos = odometry.getPosition();
        
        errors[0] = tgtX - pos.x;
        errors[1] = tgtY - pos.y;
        errors[2] = tgtRot - pos.h;
        for (int i = 0; i < 3; i++)
        {
            output[i] = pid(errors[i]);
        }
        
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
        
        if (Math.abs(output[2]) < angleLenience)
        {
            completedBools[2] = true;
        }
        else
        {
            completedBools[2] = false;
        }

        double maxXYOutput = Math.max(Math.abs(output[0]),Math.abs(output[1]));
        double maxXYHOutput = Math.max(maxXYOutput,Math.abs(output[2]));
        output[0] /= maxXYHOutput;
        output[1] /= maxXYHOutput;
        output[2] /= maxXYHOutput;

        dt.fieldOrientedTranslate(speed * output[0] * 3, speed * output[1] * 3, 0);
        // dt.fieldOrientedTranslate(speed,speed,speed);
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
    
    public double GetErrorX()
    {
        return errors[0];
    }
    
    public double GetErrorY()
    {
        return errors[1];
    }
    
    public double GetErrorH()
    {
        return errors[2];
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
} 


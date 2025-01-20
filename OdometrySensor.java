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
    
    //  <//PID TESTING LOG//>
    
    //1.55 0.32 0 = absolute holy grail :D
    
    
    
    public void init(HardwareMap hwMap)
    {
        //PositionY 
        //currently using pid on odometry position readings, could be causing some of the oscillation problems
        // might be better to use motor ticks with our odometry for more accuracy
        kp = 1.55; // best value 0.8
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
    
    double pid(double error) // it's just a p controller lol
    {
        integral += error * (runtime.seconds() / 1000);
        double derivative = (error - previous) / (runtime.seconds() * 1000);
        previous = error;
        double output = 0;
        
        output = kp * error + ki * integral + kd * derivative;
        runtime.reset();
        integral = 0;
        return output;
    }
    
    public void OdometryControl(double speed, double tgtX,double tgtY,double tgtRot)
    {
        if (!odometry.isConnected()) {return;}

        double distanceLenience = 1; //best value 1
        double angleLenience = 15; //best value 4
        
        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;
        
        pos = odometry.getPosition();
        
        errors[0] = tgtX - pos.x;
        errors[1] = tgtY - pos.y;
        errors[2] = Math.toDegrees(angleWrap(Math.toRadians(tgtRot - pos.h)));

        for (int i = 0; i < 3; i++)
        {
            output[i] = pid(errors[i]);
        }
        
        double maxXYOutput = Math.max(Math.abs(output[0]),Math.abs(output[1]));
        // double maxXYError = Math.max(Math.abs(errors[0]),Math.abs(errors[1]));
        
        if (Math.abs(errors[0]) < distanceLenience)
        {
            completedBools[0] = true;
            output[0] = 0;
        }
        else
        {
            output[0] /= maxXYOutput;
            completedBools[0] = false;
        }

        if (Math.abs(errors[1]) < distanceLenience)
        {
            completedBools[1] = true;
            output[1] = 0;
        }
        else
        {
            output[1] /= maxXYOutput;
            completedBools[1] = false;
        }
        
        if (Math.abs(errors[2]) < angleLenience)
        {
            completedBools[2] = true;
            output[2] = 0;
        }
        else
        {
            if (errors[2] < 0)
            {
                output[2] = -output[2];
            }
            output[2] /= Math.abs(output[2]);
            completedBools[2] = false;
        }
        
        // errors[0] /= maxXYError;
        // errors[1] /= maxXYError;
        // errors[2] /= errors[2];

        dt.fieldOrientedTranslate(speed * output[0], speed * output[1], speed * output[2] * 0.7, pos.h);
    }
    
    private double angleWrap(double rad)
    {
        while (rad > Math.PI)
        {
            rad -= 2 * Math.PI;
        }
        while (rad < -Math.PI)
        {
            rad += 2 * Math.PI;
        }
        return -rad;
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
    
    public double GetOutputX()
    {
        return output[0];
    }
    
    public double GetOutputY()
    {
        return output[1];
    }
    
    public double GetOutputH()
    {
        return output[2];
    }
    
    public boolean GetBoolsCompleted()
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

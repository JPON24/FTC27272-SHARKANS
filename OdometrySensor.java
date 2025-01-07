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
        ki = 0.0;
        kd = 0.0;
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
        double distanceLenience = 2;
        double angleLenience = 10;
        
        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;
        
        pos = odometry.getPosition();
        
        double[] errors = new double[3];
        errors[0] = tgtX - GetPositionX();
        errors[1] = tgtY - GetPositionY();
        errors[2] = tgtRot - GetImuReading();


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

        double maxXYOutput = Math.max(Math.abs(output[0]),Math.abs(output[1]));
        output[0] /= maxXYOutput;
        output[1] /= maxXYOutput;
        output[2] /= output[2]; // might cause oscillation from using max values although might just be confused 

        if (Math.abs(output[2]) < angleLenience)
        {
            completedBools[2] = false;
        }
        else
        {
            completedBools[2] = true;
        }

        dt.fieldOrientedTranslate(output[0] * speed, output[1] * speed, output[2] * speed);
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
}   

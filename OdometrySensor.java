package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class OdometrySensor {
    Drivetrain dt = new Drivetrain();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime dxTime = new ElapsedTime();
    ElapsedTime dyTime = new ElapsedTime();
    ElapsedTime dhTime = new ElapsedTime();
    
    SparkFunOTOS odometry; 
    SparkFunOTOS.Pose2D pos;

    boolean[] completedBools = new boolean[3];
    
    double deltaTime, last_time;
    double integralX, integralY, integralH = 0;
    double kp, ki, kdx, kdy, kdh;
    double[] output = new double[3];
    double[] errors = new double[3];
    double[] previous = new double[3];
    
    public void init(HardwareMap hwMap, boolean isAuton)
    {
        //PositionY 
        //currently using pid on odometry position readings, could be causing some of the oscillation problems
        // might be better to use motor ticks with our odometry for more accuracy
        kp = 0.075; //0.4375  0.109375 0.077
        ki = 0.08375; //0.08375  0.0209375
        kdx = 0.05; //0.18875  0.0471875
        kdy = 0.05; //0.18875  0.0471875
        kdh = 0.25; //0.25  0.0625
        last_time = 0;
        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        if (isAuton)
        {
            odometry.resetTracking();
            odometry.begin();
        }
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.setLinearScalar(40/41);
        odometry.setAngularScalar(1);
        odometry.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte)0x0D));// disables accelerometer
        odometry.setOffset(new SparkFunOTOS.Pose2D(-0.125,-3.5,0));
        dt.init(hwMap);
    }
    
    // maybe derivative not needed as velocity becomes near linear
    double pid(double error, int index) 
    {
        double output = 0;
        
        if (index == 0)
        {
            previous[0] = error;
            integralX += error * dxTime.seconds();
            double derivative = (error - previous[0]) / dxTime.seconds();
            output = kp * error + ki * integralX + kdx * derivative;
            dxTime.reset();
            integralX = 0;
        }
        else if (index == 1)
        {
            previous[1] = error;
            integralY += error * dyTime.seconds();
            double derivative = (error - previous[1]) / dyTime.seconds();
            output = kp * error + ki * integralY + kdy * derivative;
            dyTime.reset();
            integralY = 0;
        }
        else
        {
            previous[2] = error;
            integralH += error * dhTime.seconds();
            double derivative = (error - previous[2]) / dhTime.seconds();
            output = kp * error + ki * integralH + kdh * derivative;
            dhTime.reset();
            integralH = 0;
        }
        output = Range.clip(output, -1, 1);
        return output;
    }
    
    public void OdometryControl(double speed, double tgtX,double tgtY,double tgtRot)
    {
        if (!odometry.isConnected()) {return;}
        double distanceLenience = 0.6; //best value 1
        double angleLenience = 1.5; //best value 4
        
        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;
        
        pos = odometry.getPosition();
        errors[0] = tgtX - pos.x;
        errors[1] = tgtY - pos.y;
        errors[2] = Math.toDegrees(angleWrap(Math.toRadians(tgtRot - pos.h))) / 5;

        for (int i = 0; i < 3; i++)
        {
            output[i] = pid(errors[i],i);
        }
        
        double maxXYOutput = Math.max(Math.abs(output[0]),Math.abs(output[1]));
        
        if (Math.abs(errors[0]) < distanceLenience)
        {
            completedBools[0] = true;
            // output[0] = 0;
        }
        else
        {
            // output[0] /= maxXYOutput;
            completedBools[0] = false;
        }

        if (Math.abs(errors[1]) < distanceLenience)
        {
            completedBools[1] = true;
            // output[1] = 0;
        }
        else
        {
            // output[1] /= maxXYOutput;
            completedBools[1] = false;
        }
        
        if (Math.abs(errors[2]) < angleLenience)
        {
            completedBools[2] = true;
            // output[2] = 0;
        }
        else
        {
            // output[2] /= Math.abs(output[2]);
            completedBools[2] = false;
        }
        // for (int i = 0; i < 3; i++)
        // {
        //     if (Math.abs(output[i]) < 0.15)
        //     {
        //         output[i] = 0;
        //     }
        // }
        // output[0] = 0;
        // output[2] = 0;
        dt.FieldOrientedTranslate(speed * output[0], speed * output[1], 0.7 * speed * output[2], Math.toDegrees(angleWrap(Math.toRadians(pos.h))));
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
    
    public void ResetImuReadings()
    {
        odometry.resetTracking();
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

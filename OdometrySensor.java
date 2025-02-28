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
    double kpx, kpy, kph, ki, kdx, kdy, kdh;
    double pY, iY, dY;
    double xAverage, yAverage, hAverage = 0;
    double[] output = new double[3];
    double[] errors = new double[3];
    double[] previous = new double[3];

    private double angleOffset = 0;
    
    public void init(HardwareMap hwMap, boolean isAuton)
    {
        kpx = 0.28; //0.2
        kpy = 0.23; //0.2
        kph = 0.21;
        ki = 0.0; // 0.05
        kdx = 0.2; // 0.045 0.175
        kdy = 0.175; // 0.045 0.175
        kdh = 0.15; // 0.045 0.175
        
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
        odometry.setAngularScalar(239/240);
        odometry.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte)0x0D));// disables accelerometer
        odometry.setOffset(new SparkFunOTOS.Pose2D(-0.375,3.625,0));
        dt.init(hwMap);
    }

    private double LowPass(double average, double newValue)
    {
        average = (average * 0.85) + (0.15 * newValue);

        return average;
    }

    // maybe derivative not needed as velocity becomes near linear
    public double pid(double error, int index) 
    {
        double output = 0;
        
        if (index == 0)
        {
            integralX += error * dxTime.seconds();
            double derivative = (error - previous[0]) / dxTime.seconds();
            derivative = LowPass(xAverage, derivative);

            output = kpx * error + ki * integralX + kdx * derivative;
            dxTime.reset();
            previous[0] = error;

            if (previous[0] * error < 0)
            {
                integralX = 0;
            }
        }
        else if (index == 1)
        {
            integralY += error * dyTime.seconds();
            double derivative = (error - previous[1]) / dyTime.seconds();
            derivative = LowPass(yAverage, derivative);

            output = kpy * error + ki * integralY + kdy * derivative;
            dyTime.reset();

            previous[1] = error;

            if (previous[1] * error < 0)
            {
                integralY = 0;
            }
        }
        else
        {
            integralH += error * dhTime.seconds();
            double derivative = (error - previous[2]) / dhTime.seconds();
            derivative = LowPass(hAverage, derivative);

            pY = kph * error;
            iY = ki * integralH;
            dY = kdh * derivative;

            output = kph * error + ki * integralH + kdh * derivative;
            dhTime.reset();

            previous[2] = error;

            if (previous[2] * error < 0)
            {
                integralH = 0;
            }

        }
        output = Range.clip(output, -1, 1); // old coef 2*
        return output;
    }
    
    public void OdometryControl(double speed, double tgtX,double tgtY,double tgtRot)
    {
        if (!odometry.isConnected()) {return;}
        double distanceLenience = 0.75; //best value 1.75
        double angleLenience = 30; //best value 4 old 3
        
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

        completedBools[0] = Math.abs(errors[0]) < distanceLenience;

        completedBools[1] = Math.abs(errors[1]) < distanceLenience;

        completedBools[2] = Math.abs(errors[2]) < angleLenience;
        
        dt.FieldOrientedTranslate(speed * output[0], speed * output[1], speed * output[2], Math.toDegrees(angleWrap(Math.toRadians(pos.h + angleOffset))));
    }
    
    public double angleWrap(double rad)
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

    public double GetPorportionalY()
    {
        return pY;
    }

    public double GetIntegralY()
    {
        return iY;
    }

    public double GetDerivativeY()
    {
        return dY;
    }

    
    public boolean GetBoolsCompleted()
    {
        for (int i = 0; i < 3; i++)
        {
            if (!completedBools[i])
            {
                return false;
            }
        }
        return true;
    }
} 

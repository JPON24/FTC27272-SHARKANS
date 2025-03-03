package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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
    double iX, iY, iH, pX, dX;
    double xAverage, yAverage, hAverage = 0;
    double[] output = new double[3];
    double[] errors = new double[3];
    double[] previous = new double[3];

    double runtimeXSum = 0;
    double lastX = 0;

    double runtimeYSum = 0;
    double lastY = 0;

    double highestError = 0;
    
    public void init(HardwareMap hwMap, boolean isAuton)
    {
        kpx = 0.31; //0.28
        kpy = 0.23; //0.23
        kph = 0.21; // 0.21
        ki = 0; // 0
        kdx = 0.2; // 0.2
        kdy = 0.175; // 0.175
        kdh = 0.15; // 0.15
        
        last_time = 0;
        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        if (isAuton)
        {
            odometry.resetTracking();
            odometry.begin();
        }
        odometry.begin();
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.setLinearScalar(1);
        odometry.setAngularScalar(1);
//        odometry.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte)0x0B));
        odometry.setOffset(new SparkFunOTOS.Pose2D(0.4375,3.625,0));
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

            if (previous[0] * error < 0)
            {
                integralX = 0;
            }

            double derivative = (error - previous[0]) / dxTime.seconds();
            derivative = LowPass(xAverage, derivative);
            iX = integralX;

            output = kpx * error + ki * integralX + kdx * derivative;
            dxTime.reset();
            previous[0] = error;
        }
        else if (index == 1)
        {
            integralY += error * dyTime.seconds();

            if (previous[1] * error < 0)
            {
                integralY = 0;
            }

            double derivative = (error - previous[1]) / dyTime.seconds();
            derivative = LowPass(yAverage, derivative);
            iY = integralY;

            output = kpy * error + ki * integralY + kdy * derivative;
            dyTime.reset();

            previous[1] = error;
        }
        else
        {
            integralH += error * dhTime.seconds();

            if (previous[2] * error < 0)
            {
                integralH = 0;
            }

            double derivative = (error - previous[2]) / dhTime.seconds();
            derivative = LowPass(hAverage, derivative);

            pX = kph * error;
            dX = derivative;
            iH = integralH;

            output = kph * error + ki * integralH + kdh * derivative;
            dhTime.reset();

            previous[2] = error;

        }
        output = Range.clip(output, -1, 1); // old coef 2*
        return output;
    }

    private void Integrals()
    {
        runtimeXSum += Math.abs(GetPositionX() - lastX);
        lastX = GetPositionX();

        runtimeYSum += Math.abs(GetPositionY() - lastY);
        lastY = GetPositionY();
    }
    
    public void OdometryControl(double speed, double tgtX,double tgtY,double tgtRot)
    {
        if (!odometry.isConnected()) {return;}
        double distanceLenience = 0.5; //best value 1.75
        double angleLenience = 100; //best value 4 old 3
        
        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;
        
        pos = odometry.getPosition();

        Integrals();

        errors[0] = tgtX - pos.x;
        errors[1] = tgtY - pos.y;
        if (Math.abs(errors[0]) > highestError)
        {
            highestError = Math.abs(errors[0]);
        }
        if (Math.abs(errors[1]) > highestError)
        {
            highestError = Math.abs(errors[1]);
        }

        errors[2] = (Math.toDegrees(angleWrap(Math.toRadians(tgtRot - pos.h)))) / 10;
//        for (int i = 0; i < 3; i++)
//        {
//            output[i] = pid(errors[i],i);
//        }

        // normalized against one another
        // should create weird diagonal movement
        // might have to add increased magnitude to error, currently between -1 and 1
        output[0] = pid(errors[0]/highestError,0);
        output[1] = pid(errors[1]/highestError, 1);
        output[2] = pid(errors[2],2);

        // could add non linearity with sin?
        // * math.pi to convert value to radians
        output[0] = Math.sin(output[0] * Math.PI);
        output[1] = Math.sin(output[1] * Math.PI);

        completedBools[0] = Math.abs(errors[0]) < distanceLenience;
        completedBools[1] = Math.abs(errors[1]) < distanceLenience;
        completedBools[2] = Math.abs(errors[2]) < angleLenience;

        // almost!! non linearity is nice but oscillates heavily
//        double sigmoidX = Sigmoid(output[0] / highestOutput) * output[0]/Math.abs(output[0]);
//        double sigmoidY = Sigmoid(output[1] / highestOutput) * output[0]/Math.abs(output[0]);
//        double tanHX = Math.tanh(output[0]/pid(highestError,0));
//        double tanHY = Math.tanh(output[1]/pid(highestError,1));
//        double sinX = Math.sin(output[0]);
//        double sinY = Math.sin(output[1]);

        dt.FieldOrientedTranslate(speed * output[0], speed * output[1], speed * output[2], pos.h);
    }

    private double Sigmoid(double x)
    {
        return 1 / Math.pow(2.718,-x);
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

    public void SetRuntimeXSum(double temp)
    {
        runtimeXSum = temp;
    }

    public double GetIntegralSumX() {return runtimeXSum;}

    public double GetIntegralSumY() {return runtimeYSum;}

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

    public double GetDerivativeX() { return dX; }

    public double GetPorportionalX() {return pX;}
    
    public double GetOutputY()
    {
        return output[1];
    }
    
    public double GetOutputH()
    {
        return output[2];
    }

    public double GetIntegralX()
    {
        return iX;
    }

    public double GetIntegralY()
    {
        return iY;
    }

    public double GetIntegralH()
    {
        return iH;
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
        highestError = 0;
        return true;
    }
} 

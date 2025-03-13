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
    double kpx, kpy, kph, kix, kiy, kih, kdx, kdy, kdh;
    double iX, iY, iH, pX, dX;
    double xAverage, yAverage, hAverage = 0;
    double[] output = new double[3];
    double[] errors = new double[3];
    double[] previous = new double[3];

    boolean auton = false;

    double runtimeXSum = 0;
    double lastX = 0;

    double runtimeYSum = 0;
    double lastY = 0;

    double maximumOutputX = 1;
    double maximumOutputY = 1;

    double diagonalScalar = 0;

    public void init(HardwareMap hwMap, boolean isAuton) {
//        kpx = 0.38; //0.38
//        kpy = 0.24; //0.24
//        kph = 0.34; // 0.21
//        kix = 0.01; // 0.01
//        kiy = 0; //  0
//        kih = 0.01; // 0.43
//        kdx = 0.25; // 0.28
//        kdy = 0.15; // 0.21
//        kdh = 0.25; // 0.15

        TuningDown();

        last_time = 0;
        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        if (isAuton) {
            odometry.resetTracking();
            odometry.begin();
        }
        auton = isAuton;
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.calibrateImu();
        odometry.setLinearScalar(1);
        odometry.setAngularScalar(1);
//        odometry.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte)0x0B));
        // remeasure
        odometry.setOffset(new SparkFunOTOS.Pose2D(0.4375, 3.625, 0));
        dt.init(hwMap);
    }

    // gear ratio moving from 1.25 to 1.4
    // scalar of 1.123

    public void TuningDown() {
        kpx = 0.38; //0.38 1.14
        kpy = 0.2; //0.2 0.6
        kph = 0.34; // 0.34 1.12
        kix = 0.0; // 0.01
        kiy = 0.0; // 0.01
        kih = 0.01 ; // 0.01
        kdx = 0.32; // 0.32 0.16
        kdy = 0.13; // 0.13 0.065
        kdh = 0.25; // 0.25 0.25
    }

    public void TuningUp() {
        kpx = 0.38; //0.38
        kpy = 0.24; //0.24
        kph = 0.34; // 0.34
        kix = 0; // 0.01
        kiy = 0; //  0
        kih = 0.01; // 0.43
        kdx = 0.25; // 0.25
        kdy = 0.15; // 0.15
        kdh = 0.25; // 0.25
    }

    private double LowPass(double average, double newValue) {
        average = (average * 0.85) + (0.15 * newValue);

        return average;
    }

    // maybe derivative not needed as velocity becomes near linear
    public double pid(double error, int index, double distanceLenience) {
        double output = 0;
        if (index == 0) {
            integralX += error * dxTime.seconds();

            if (previous[0] * error < 0) {
                integralX = 0;
            }
            iX = integralX;

            double derivative = (error - previous[0]) / dxTime.seconds();
            derivative = LowPass(xAverage, derivative);

            output = kpx * error + kix * integralX + kdx * derivative;
            dxTime.reset();
            previous[0] = error;

            NewHighestOutputX(Math.abs(output));
            output = Range.clip(output, -1, 1); // old coef 2*
            if (Math.max(Math.abs(maximumOutputX), Math.abs(maximumOutputY)) == Math.abs(maximumOutputY) && error > Math.abs(0.5)) {
                output = DiagonalScalar(Math.abs(maximumOutputX), Math.abs(maximumOutputY), 0.35) * output / Math.abs(output);
            }
        } else if (index == 1) {
            integralY += error * dyTime.seconds();

            if (previous[1] * error < 0) {
                integralY = 0;
            }

            double derivative = (error - previous[1]) / dyTime.seconds();
            derivative = LowPass(yAverage, derivative);
            iY = integralY;
            pX = kph * error;
            dX = derivative;

            output = kpy * error + kiy * integralY + kdy * derivative;
            dyTime.reset();

            previous[1] = error;
            NewHighestOutputY(Math.abs(output));
            output = Range.clip(output, -1, 1); // old coef 2*

            if (Math.max(Math.abs(maximumOutputX), Math.abs(maximumOutputY)) == Math.abs(maximumOutputX) && error > Math.abs(0.5)) {
                output *= DiagonalScalar(Math.abs(maximumOutputX), Math.abs(maximumOutputY), 0.3) * output / Math.abs(output);
            }
        } else {
            integralH += error * dhTime.seconds();

            if (previous[2] * error < 0) {
                integralH = 0;
            }

            double derivative = (error - previous[2]) / dhTime.seconds();
            derivative = LowPass(hAverage, derivative);

            iH = integralH;

            output = kph * error + kih * integralH + kdh * derivative;
            dhTime.reset();

            previous[2] = error;

            output = Range.clip(output, -1, 1); // old coef 2*
        }
//        output = Range.clip(output, -1, 1); // old coef 2*
        return output;
    }

    private void Integrals() {
        runtimeXSum += Math.abs(GetPositionX() - lastX);
        lastX = GetPositionX();

        runtimeYSum += Math.abs(GetPositionY() - lastY);
        lastY = GetPositionY();
    }

    public void OdometryControl(double speed, double tgtX, double tgtY, double tgtRot, double distanceLenience, int axis) {
        if (!odometry.isConnected()) {
            return;
        }
//        distanceLenience; //best value 1.75
        double angleLenience = 60; //best value 15

        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;

        pos = odometry.getPosition();

        Integrals();
        if (auton) {
            tgtX += runtimeXSum * 0.0164;
            tgtY -= runtimeYSum * 0.005;
        }

        errors[0] = tgtX - pos.x;
        errors[1] = tgtY - pos.y;
        errors[2] = (Math.toDegrees(angleWrap(Math.toRadians(tgtRot - pos.h)))) / 10;

        if (new ArmLiftMotor().GetLocalNeutral() == 1250) {
            TuningUp();
        } else {
            TuningDown();
        }

        // normalized against one another
        // should create weird diagonal movement
        // might have to add increased magnitude to error, currently between -1 and 1
        output[0] = pid(errors[0], 0, distanceLenience);
        output[1] = pid(errors[1], 1, distanceLenience);
        output[2] = pid(errors[2], 2, distanceLenience);

        if (tgtRot == 1) {
            completedBools[2] = true;
            output[2] = 0;
        } else {
            completedBools[2] = Math.abs(errors[2]) < angleLenience;
        }

        completedBools[0] = Math.abs(errors[0]) < distanceLenience;
        completedBools[1] = Math.abs(errors[1]) < distanceLenience;


        if (axis == 0) {
            output[1] = output[1] / Math.abs(output[1]) * 0.2;
            completedBools[1] = true;
        } else if (axis == 1) {
            output[0] *= 0.4;
            completedBools[0] = true;
        }

        dt.FieldOrientedTranslate(speed * output[0], speed * output[1], speed * output[2], pos.h);
    }

    private void NewHighestOutputX(double x) {
        if (x > maximumOutputX) {
            maximumOutputX = x;
        }
    }

    private void NewHighestOutputY(double y) {
        if (y > maximumOutputY) {
            maximumOutputY = y;
        }
    }

    private double DiagonalScalar(double x, double y, double min) {
        diagonalScalar = Math.max((Math.min(x, y) / Math.max(x, y)), min);
        return diagonalScalar;
    }

    public void Rehome()
    {
        odometry.resetTracking();
    }

    public double GetDiagonalScalar()
    {
        return diagonalScalar;
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
        integralX = 0;
        integralY = 0;
        maximumOutputX = 1;
        maximumOutputY = 1;
        return true;
    }
} 

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.Drivetrain;
//import com.qualcomm.robotcore.util.ElapsedTime;


public class Sensor1 {
    Drivetrain dt = new Drivetrain();
    ActionHandler action = new ActionHandler();
    //ElapsedTime runtime = new ElapsedTime();
    
    MaxSonarI2CXL leftSensor = null;
    MaxSonarI2CXL backSensor = null;
    MaxSonarI2CXL rightSensor = null;
    
    boolean calibrating = false;
    
    double initialDistanceL = 0;
    double initialDistanceB = 0;
    double initialDistanceR = 0;
    
    public void init(HardwareMap hwMap)
    {
        leftSensor = hwMap.get(MaxSonarI2CXL.class, "Sensor1");
        backSensor = hwMap.get(MaxSonarI2CXL.class, "Sensor2");
        rightSensor = hwMap.get(MaxSonarI2CXL.class, "Sensor3");
        dt.init(hwMap);
    }
    
    public void Calibrate()
    {
        calibrating = true;
        initialDistanceL = leftSensor.getDistanceSync(DistanceUnit.INCH);
        initialDistanceB = backSensor.getDistanceSync(DistanceUnit.INCH);
        initialDistanceR = rightSensor.getDistanceSync(DistanceUnit.INCH);
    }
    
    /*private void ElapsedCalls()
    {
        double seconds = runtime.seconds();
        if (seconds >= 14.5 && seconds <= 15.5)
        {
            
        }
        
    }*/
    
    //dir: 0 = up; speed = up; 2 = down; 3 = right;
    public void moveToPositionR(double targetPosition,double speed, char dir,int id)
    {
        if(!calibrating)
        {
            Calibrate();
        }
        double currentDistanceR = rightSensor.getDistanceSync(DistanceUnit.INCH);
        double currentDistanceB = backSensor.getDistanceSync(DistanceUnit.INCH);
        if(dir == 'F' || dir == 'B')//forward and back
        {
            switch(dir)
            {
                case 'F':
                    if(currentDistanceB < initialDistanceB + targetPosition)
                    {
                        //currentDistanceB = backSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(0,speed,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
                case 'B':
                    if (currentDistanceB > initialDistanceB - targetPosition)
                    {
                        //currentDistanceB = backSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(0,-speed,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
            }
        }
        else
        {
            switch(dir)
            {
                case 'L':
                    if (currentDistanceR > initialDistanceR - targetPosition)
                    {
                        //currentDistanceS = sideSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(-speed,0,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
                case 'R':
                    if (currentDistanceR < initialDistanceR + targetPosition)
                    {
                        //currentDistanceS = sideSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(speed,0,0); 
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
            }
        }
    }
    
    public void moveToPositionL(double targetPosition,double speed, char dir,int id)
    {
        if(!calibrating)
        {
            Calibrate();
        }
        double currentDistanceL = leftSensor.getDistanceSync(DistanceUnit.INCH);
        double currentDistanceB = backSensor.getDistanceSync(DistanceUnit.INCH);
        if(dir == 'F' || dir == 'B')//forward and back
        {
            switch(dir)
            {
                case 'F':
                    if(currentDistanceB < initialDistanceB + targetPosition)
                    {
                        //currentDistanceB = backSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(0,-speed,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        calibrating = false;
                        action.IncrementActionId();
                    }
                    break;
                case 'B':
                    if (currentDistanceB > initialDistanceB - targetPosition)
                    {
                        //currentDistanceB = backSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(0,speed,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        calibrating = false;
                        action.IncrementActionId();
                    }
                    break;
            }
        }
        else
        {
            switch(dir)
            {
                case 'L':
                    if (currentDistanceL > initialDistanceL - targetPosition)
                    {
                        //currentDistanceS = sideSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(speed,0,0);
                    }
                    else
                    {
                        dt.fieldOrientedTranslate(0,0,0);
                        calibrating = false;
                        action.IncrementActionId();
                    }
                    break;
                case 'R':
                    if (currentDistanceL < initialDistanceL + targetPosition)
                    {
                        //currentDistanceS = sideSensor.getDistance(DistanceUnit.INCH);
                        dt.fieldOrientedTranslate(-speed,0,0); 
                    }
                    else
                    {
                        calibrating = false;
                        dt.fieldOrientedTranslate(0,0,0);
                        action.IncrementActionId();
                    }
                    break;
            }
        }
    }
}

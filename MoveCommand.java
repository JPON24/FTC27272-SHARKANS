package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.ClawServo;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;
import org.firstinspires.ftc.teamcode.OdometrySensor;
import org.firstinspires.ftc.teamcode.CommandSystem;

public class MoveCommand  {
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    OdometrySensor s1 = new OdometrySensor();
    CommandSystem command = new CommandSystem();

    int lastA = 0;
    
    public void init(HardwareMap hwMap, boolean isAuton)
    {
        dt.init(hwMap);
        cs.init(hwMap, true);
        am.init(hwMap);
        s1.init(hwMap, isAuton);
    }

    public void MoveToPosition(double speed, double tgtX, double tgtY, double rot, int tgtE, int tgtA, boolean tgtClaw, char tgtWrist)
    {
        // reset for next command
        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();

        // movement, will be driven by s1.GetBoolsCompleted()
        command.SetElementFalse('m'); 

        if (tgtA != lastA) // if arm movement must occur
        {
            command.SetElementFalse('a');
        }

//        command.SetElementFalse('c');
        lastA = tgtA;
        
        s1.OdometryControl(speed,tgtX,tgtY,rot);

        localCopy = command.GetMap();
        
        while (!command.GetBoolsCompleted())
        {
            cs.SetWristMode(tgtWrist);
            // for every key (m, e, a, c, w)
            for (Character key : localCopy.keySet())
            {
                // like an if else but more efficient
                // if a subsystem has reached it's position, set it to complete
                // otherwise, set it to false and stop moving it 
                switch (key)
                {
                    case 'm':
                        s1.OdometryControl(speed,tgtX,tgtY,rot);
                        if (s1.GetBoolsCompleted())
                        {
                            command.SetElementTrue('m');
                            // dt.FieldOrientedTranslate(0,0,0,0);
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('m');
                            break;
                        }
                    case 'a':
                        am.rotate(tgtA,'A');
                        if (am.GetCompleted(tgtA))
                        {
                            command.SetElementTrue('a');
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('a');
                            break;
                        }
//                    case 'c':
//                        if (cs.GetClawClosed() == tgtClaw)
//                        {
//                            command.SetElementTrue('c');
//                            break;
//                        }
//                        else
//                        {
//                            command.SetElementFalse('c');
//                            cs.clawMove(tgtClaw);
//                            break;
//                        }
                }
            }

            cs.Update();
        }
    }

    public void MoveToPositionCancellable(double speed, double x, double y, double h, int tgtA, boolean tgtClaw, char tgtWrist)
    {
        // reset for next command
        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();

        command.SetElementFalse('m');
        command.SetElementFalse('a');

//        if (tgtClaw != cs.GetClawClosed()) // if move has to happen
//        {
//            command.SetElementFalse('c');
//        }

        localCopy = command.GetMap();

        // for every key (m, e, a, c, w)
        for (Character key : localCopy.keySet())
        {
            cs.SetWristMode(tgtWrist);
            // like an if else but more efficient
            // if a subsystem has reached it's position, set it to complete
            // otherwise, set it to false and stop moving it 
            switch (key)
            {
                case 'm':
                    s1.OdometryControl(speed,x,y,h);
                    if (s1.GetBoolsCompleted())
                    {
                        command.SetElementTrue('m');
                        // dt.FieldOrientedTranslate(0,0,0,0);
                        break;
                    }
                    else
                    {
                        command.SetElementFalse('m');
                        break;
                    }
                case 'a':
                    am.rotate(tgtA,'A');
                    if (am.GetCompleted(tgtA))
                    {
                        command.SetElementTrue('a');
                        break;
                    }
                    else
                    {
                        command.SetElementFalse('a');
                        break;
                    }
//                case 'c':
//                    if (cs.GetClawClosed() == tgtClaw)
//                    {
//                        command.SetElementTrue('c');
//                        break;
//                    }
//                    else
//                    {
//                        command.SetElementFalse('c');
//                        cs.clawMove(tgtClaw);
//                        break;
//                    }
            }
        }
        cs.Update();
    }

    public boolean GetCommandState()
    {
        return command.GetBoolsCompleted();
    }
    
    public double GetClawPositionReading()
    {
        return cs.GetClawPosition();
    }
    
    public void UpdateClawPosition()
    {
        cs.Update();
    }
}

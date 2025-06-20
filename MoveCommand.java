package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveCommand  {
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    SharkDrive shark = new SharkDrive();
    CommandSystem command = new CommandSystem();
    ElapsedTime timeout = new ElapsedTime();
    Extension extend = new Extension();

    int lastA = 0;
    int lastE = 0;
    
    public void init(HardwareMap hwMap, boolean isAuton)
    {
        dt.init(hwMap);
        cs.init(hwMap, true);
        am.init(hwMap);
        extend.init(hwMap);
        shark.init(hwMap, isAuton);
    }

    public void MoveToPosition(double speed, double tgtX, double tgtY, double rot, double d, int axis, double speedA, int tgtA, int tgtE, double roll, double pitch, boolean tgtClaw, char tgtWrist)
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
        if (tgtE != lastE)
        {
            command.SetElementFalse('e');
        }

        lastA = tgtA;
        lastE = tgtE;
        
        shark.OdometryControl(speed,tgtX,tgtY,rot,d,axis);

        localCopy = command.GetMap();

        cs.SetWristMode(tgtWrist);
        cs.SetClawOpen(tgtClaw);
        am.SetArmSpeed(speedA);
        extend.SetLocalManipulatorState(roll, pitch);

        timeout.reset();

        while (!command.GetBoolsCompleted()) {
            // for every key (m, e, a, c, w)
            for (Character key : localCopy.keySet()) {
                // like an if else but more efficient
                // if a subsystem has reached it's position, set it to complete
                // otherwise, set it to false and stop moving it 
                switch (key) {
                    case 'm':
                        if (shark.GetBoolsCompleted()) {
                            shark.OdometryControl(0, tgtX, tgtY, rot, d, axis);
                            command.SetElementTrue('m');
                            // dt.FieldOrientedTranslate(0,0,0,0);
                            break;
                        }
                        else
                        {
                            shark.OdometryControl(speed, tgtX, tgtY, rot, d, axis);
                            command.SetElementFalse('m');
                            break;
                        }
                    case 'a':
                        am.Rotate(tgtA, 'A');
                        if (am.GetCompleted(tgtA)) {
                            command.SetElementTrue('a');
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('a');
                            break;
                        }
                    case 'e':
                        extend.MoveExtend(tgtE,'a');
                        if (extend.GetCompleted(tgtE))
                        {
                            command.SetElementTrue('e');
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('e');
                            break;
                        }
                }
            }

            if (timeout.seconds() > 3.5)
            {
                break;
            }

            cs.Update();
        }
    }

    public void MoveToPositionCancellable(double speed, double x, double y, double h, double d, int axis, double speedA, int tgtA, int tgtE, double roll, double pitch, boolean tgtClaw, char tgtWrist)
    {
        // reset for next command
        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();

        command.SetElementFalse('m');
        command.SetElementFalse('a');
        command.SetElementFalse('e');

        localCopy = command.GetMap();

        cs.SetWristMode(tgtWrist);
        cs.SetClawOpen(tgtClaw);
        am.SetArmSpeed(speedA);
        am.Rotate(tgtA,'A');
        extend.MoveExtend(tgtE, 'a');
        extend.SetLocalManipulatorState(roll, pitch);

        // for every key (m, e, a, c, w)
        for (Character key : localCopy.keySet())
        {
            // like an if else but more efficient
            // if a subsystem has reached it's position, set it to complete
            // otherwise, set it to false and stop moving it 
            switch (key)
            {
                case 'm':
                    if (shark.GetBoolsCompleted())
                    {
                        shark.OdometryControl(0,x,y,h,d,axis);
                        command.SetElementTrue('m');
                        // dt.FieldOrientedTranslate(0,0,0,0);
                        break;
                    }
                    else
                    {
                        shark.OdometryControl(speed,x,y,h,d,axis);
                        command.SetElementFalse('m');
                        break;
                    }
                case 'a':
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
                case 'e':
                    extend.MoveExtend(tgtE,'a');
                    if (extend.GetCompleted(tgtE))
                    {
                        command.SetElementTrue('e');
                        break;
                    }
                    else
                    {
                        command.SetElementFalse('e');
                        break;
                    }
            }
        }
        cs.Update();
    }
    public void MoveToPositionCV(double speed, double x, double y, double h, double d, int axis, double speedA, int tgtA, int tgtE, boolean tgtClaw, double roll, double pitch, int cx, double dist)
    {
        // reset for next command
        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();

        command.SetElementFalse('m');
        command.SetElementFalse('a');
        command.SetElementFalse('e');

        localCopy = command.GetMap();

//        cs.SpecifyDiffPos(wristL,wristR);
        cs.SetClawOpen(tgtClaw);
        am.SetArmSpeed(speedA);
        am.Rotate(tgtA,'A');
        extend.MoveExtend(tgtE, 'a');
        extend.SetLocalManipulatorState(roll, pitch);

        // for every key (m, e, a, c, w)
        for (Character key : localCopy.keySet())
        {
            // like an if else but more efficient
            // if a subsystem has reached it's position, set it to complete
            // otherwise, set it to false and stop moving it
            switch (key)
            {
                case 'm':
                    if (shark.GetBoolsCompleted())
                    {
                        shark.CVControl(0,x,y,h,d,axis,cx,dist);
                        command.SetElementTrue('m');
                        // dt.FieldOrientedTranslate(0,0,0,0);
                        break;
                    }
                    else
                    {
                        shark.CVControl(speed,x,y,h,d,axis,cx,dist);
                        command.SetElementFalse('m');
                        break;
                    }
                case 'a':
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
                case 'e':
                    extend.MoveExtend(tgtE,'a');
                    if (extend.GetCompleted(tgtE))
                    {
                        command.SetElementTrue('e');
                        break;
                    }
                    else
                    {
                        command.SetElementFalse('e');
                        break;
                    }
            }
        }
        cs.Update();
    }

    public boolean GetCommandState()
    {
        return command.GetBoolsCompleted();
    }
}

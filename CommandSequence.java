package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.HashMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.ClawServo;
import org.firstinspires.ftc.teamcode.Extension_1;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;
import org.firstinspires.ftc.teamcode.OdometrySensor;
import org.firstinspires.ftc.teamcode.CommandSystem;

@Autonomous
public class CommandSequence extends LinearOpMode
{
    ElapsedTime runtime = new ElapsedTime();
    
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    Extension_1 e1 = new Extension_1();
    OdometrySensor s1 = new OdometrySensor();
    CommandSystem command = new CommandSystem();
    
    boolean moving = true;
    double speed = 0.9;
    int currentAction = 1;

    double halfWidth = 7.625;
    double halfLength = 7.5;

    int lastE = 0;
    int lastA = 0;


    
    // use later for scalability
    // double halfLength = 8.625; //y
    // double halfWidth = 7.625; //x
    
    private void ContinueMovement()
    {
        moving = true;
    }
    
    private void MoveToPosition(double speed, double tgtX, double tgtY, double rot, int tgtE, int tgtA, boolean tgtClaw, char tgtWrist)
    {
        // reset for next command
        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();

        // movement, will be driven by s1.GetBoolsCompleted()
        command.SetElementFalse('m'); 

        // dont use 0 for minimum position of arm and extension because of this, use 1 instead
        if (tgtE != lastE) // if extension must occur
        {
            command.SetElementFalse('e');
        }
        if (tgtA != lastA) // if arm movement must occur
        {
            command.SetElementFalse('a');
        }
        if (tgtClaw != cs.GetClawPosition()) // if move has to happen
        {
            command.SetElementFalse('c');
        }
        if (tgtWrist != cs.GetWristState()) // if move has to happen
        {
            command.SetElementFalse('w');
        }
        lastE = tgtE;
        lastA = tgtA;
        
        s1.OdometryControl(speed,tgtX,tgtY,rot);
        localCopy = command.GetMap();
        
        while (!command.GetBoolsCompleted())
        {
            // for every key (m, e, a, c, w)
            for (Character key : localCopy.keySet())
            {
                // like an if else but more efficient
                // if a subsystem has reached it's position, set it to complete
                // otherwise, set it to false and stop moving it 
                switch (key)
                {
                    case 'm':
                        if (s1.GetBoolsCompleted())
                        {
                            command.SetElementTrue('m');
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('m');
                            s1.OdometryControl(speed,tgtX,tgtY,rot);
                            break;
                        }
                    case 'e':
                        if (e1.GetCompleted(tgtE))
                        {
                            command.SetElementTrue('e');
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('e');
                            e1.Move(speed,tgtE,'A');
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
                            am.rotate(tgtA,'A');
                            break;
                        }
                    case 'c':
                        if (cs.GetClawPosition() == tgtClaw)
                        {
                            command.SetElementTrue('c');
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('c');
                            cs.clawMove(tgtClaw);
                            break;
                        }
                    case 'w':
                        if (cs.GetWristState() == tgtWrist)
                        {
                            command.SetElementTrue('w');
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('w');
                            cs.setWristMode(tgtWrist);
                            break;
                        }
                }
            }
            
            telemetry.addData("current action",currentAction);
            telemetry.addData("volts normalized",hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage());
            telemetry.addData("speed",speed);
            telemetry.addData("x",s1.GetPositionX());
            telemetry.addData("y",s1.GetPositionY());
            telemetry.addData("h",s1.GetImuReading());
            // telemetry.addData("tgtx", tgtX);
            // telemetry.addData("tgty", tgtY);
            // telemetry.addData("tgth", rot);
            // telemetry.addData("error x",s1.GetErrorX());
            // telemetry.addData("error y",s1.GetErrorY());
            // telemetry.addData("error h",s1.GetErrorH());
            // telemetry.addData("output x",s1.GetOutputX());
            // telemetry.addData("output y",s1.GetOutputY());
            // telemetry.addData("output h",s1.GetOutputH());
            telemetry.addData("position a",am.GetCurrentPosition());
            telemetry.addData("position e",e1.extension_1.getCurrentPosition());
            telemetry.addData("position claw",cs.GetClawPosition());
            telemetry.addData("position wrist",cs.GetWristState());
            telemetry.Update();
            UpdateClaw();
        }
        runtime.reset();
        currentAction += 1;
    }

    private void Stop()
    {
        dt.FieldOrientedTranslate(0,0,0,s1.GetImuReading());
        moving = false;
    }
    
    private void UpdateClaw()
    {
        cs.Update();
    }
    
    private void CommandSequence()
    {
        /* Parameter order --
        double speed : range 0 - 1
        double x position : range > 0
        double y position : range > 0
        double rotation : range 0 - 360
        int tgtExtension : range 0 - 800
        int tgtArmRotation : range -15, 5, 30, 45, 50, 60, 70, 80, 90
        boolean tgtClawOpen
        char wristPosition : range 'N', 'D', 'B'
        */
        
        // length = 15 inches
        // width = 15.25 inches
        // height = 15 inches
        // 260 degree ROM
        
        // hook
        MoveToPosition(speed,0,-21,0,0,0,false,'N'); //1
        MoveToPosition(speed,0,-19,0,0,0,false,'N'); //2
        MoveToPosition(speed,0,-19,0,0,0,true,'N'); //3
        
        // push
        MoveToPosition(speed,-26,-19,0,0,0,false,'N'); //4
        MoveToPosition(speed,-26,-48,0,0,0,true,'N'); //5
        MoveToPosition(speed,-37,-48,0,0,0,true,'N'); //6
        MoveToPosition(speed,-37,-8.5,0,0,0,true,'N'); //7

        // grab
        MoveToPosition(speed,-37,-8.5,0,0,0,false,'N'); //8
        
        // hook
        MoveToPosition(speed,0,-21,0,0,0,false,'N'); //9
        MoveToPosition(speed,0,-19,0,0,0,false,'N'); //2
        MoveToPosition(speed,0,-19,0,0,0,true,'N'); //10
        
        // push
        MoveToPosition(speed,-36,-19,0,0,0,false,'N'); //11
        MoveToPosition(speed,-36,-48,0,0,0,false,'N'); //11
        MoveToPosition(speed,-47,-48,0,0,0,false,'N'); //12
        MoveToPosition(speed,-47,-8.5,0,0,0,false,'N'); //13
        
        // grab
        MoveToPosition(speed,-47,-8.5,0,0,0,false,'N'); //14
    
        // hook
        MoveToPosition(speed,0,-21,0,0,0,false,'N'); //15
        MoveToPosition(speed,0,-19,0,0,0,false,'N'); //2
        MoveToPosition(speed,0,-19,0,0,0,true,'N'); //16

        // push
        MoveToPosition(speed,-46,-19,0,0,0,false,'N'); //11
        MoveToPosition(speed,-46,-48,0,0,0,false,'N'); //17
        MoveToPosition(speed,-52,-48,0,0,0,false,'N'); //18
        MoveToPosition(speed,-52,-8.5,0,0,0,false,'N'); //19

        // grab
        MoveToPosition(speed,-52,-8.5,0,0,0,false,'N'); //20

        // hook
        MoveToPosition(speed,0,-21,0,0,0,false,'N'); //21
        MoveToPosition(speed,0,-19,0,0,0,false,'N'); //2
        MoveToPosition(speed,0,-19,0,0,0,true,'N'); //22

        //grab
        MoveToPosition(speed,-27.5,-8.5,0,0,0,false,'N'); //23

        // hook
        MoveToPosition(speed,0,-21,0,0,0,false,'N'); //24
        MoveToPosition(speed,0,-19,0,0,0,false,'N'); //2
        MoveToPosition(speed,0,-19,0,0,0,true,'N'); //25

        // end sequence
        MoveToPosition(speed,-35,-4,0,0,0,true,'N');
        Stop();
    }
    
    private void TestSequence()
    {
        MoveToPosition(speed,0,0,180,0,0,false,'N');
    }
    
    @Override
    public void runOpMode() 
    {
        dt.init(hardwareMap);
        cs.init(hardwareMap);
        am.init(hardwareMap);
        e1.init(hardwareMap);
        s1.init(hardwareMap);
        
        double volts = hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        double normalizedVolts = (1 - volts/14) + 0.57; // reduced to account for volt drops during auton
        speed *= normalizedVolts;
        
        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                CommandSequence();
                // TestSequence();
            }
        }
    }
}


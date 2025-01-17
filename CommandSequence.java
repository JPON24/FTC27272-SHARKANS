package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
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

import java.util.*;

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
    
    // double output[] = new double[3];
    
    
    boolean moving = true;
    double speed = 0.6;
    int currentAction = 1;
    
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
        if (tgtE != 0) // if extension must occur
        {
            command.SetElementFalse('e');
        }
        if (tgtA != 0) // if arm movement must occur
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
        
        localCopy = command.GetMap();
        while (!command.GetBoolsCompleted())
        {
            // movement done
            if (s1.GetBoolsCompleted())
            {
                command.SetElementTrue('m');
            }

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
                            e1.move(speed,tgtE,"A",0);
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
                            am.rotate(tgtA,"A",0);
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
            telemetry.addData("error x",s1.GetErrorX());
            telemetry.addData("error y",s1.GetErrorY());
            telemetry.addData("error h",s1.GetErrorH());
            telemetry.addData("output x",s1.GetOutputX());
            telemetry.addData("output y",s1.GetOutputY());
            telemetry.addData("output h",s1.GetOutputH());
            telemetry.addData("output a",am.armLift.getCurrentPosition());
            telemetry.addData("output e",e1.extension_1.getCurrentPosition());
            telemetry.addData("output claw",cs.GetClawPosition());
            telemetry.addData("output wrist",cs.GetWristState());
            telemetry.update();
            UpdateClaw();
        }
        
        runtime.reset();
        // Stop();
        currentAction += 1;
    }

    private void Stop()
    {
        dt.fieldOrientedTranslate(0,0,0,s1.GetImuReading());
        moving = false;
    }
    
    private void OpenClaw()
    {
        cs.clawMove(false);
        UpdateClaw();
        sleep(100);
    }
    private void CloseClaw()
    {
        cs.clawMove(true);
        UpdateClaw();
        sleep(100);
    }
    
    private void Extend(double power, int tgt)
    {
        UpdateClaw();
        //if (action.GetActionId() != id) {return;}
        e1.move(power,tgt,"A",0);
    }
    private void Retract(double power, int tgt)
    {
        UpdateClaw();
        //if(action.GetActionId()!=id){return;}
        e1.move(-power,tgt,"A",0);
    }
    
    private void RotateArm(double rotation)
    {
        UpdateClaw();
        am.rotate(rotation,"A",0);
    }
    
    private void UpdateClaw()
    {
        cs.update();
    }
    
    
    /*
     * will require updating - adding the extra parameters for other movements
     * refer to method for determining what values to add
     */
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
        char wristPosition : range 'N', 'D'
        */
        
        //at 0.6 speed MoveToPosition 22 actually means MoveToPosition 31
        //at 0.6 speed MoveToPosition 10 actually means MoveToPosition 22
        MoveToPosition(speed,0,22,0,750,90,false,'N'); // 1
        MoveToPosition(speed,0,22,0,650,90,false,'N'); // 1
        // MoveToPosition(speed,0,20,0,650,60,false,'N'); //1
        // MoveToPosition(speed,0,20,0,650,60,true,'N'); //1
        // MoveToPosition(speed,48,8,180,20,30,true,'N');// 2
        // MoveToPosition(speed,48,8,180,20,30,false,'N');//2
        // MoveToPosition(speed,0,22,0,800,90,false,'N'); // 3
        // MoveToPosition(speed,0,22,0,650,60,false,'N'); // 6 
        // MoveToPosition(speed,0,20,0,650,60,true,'N'); // 7
        
        
        // MoveToPosition(speed,38,18,0); // 9
        // MoveToPosition(speed,38,58.625,0); // 10 
        // MoveToPosition(speed,49,58.625,0); // 11
        // MoveToPosition(speed,49,8,0); // 12
        
        // MoveToPosition(speed,49,18,0); // 13 
        // MoveToPosition(speed,49,58.625,0); // 14 
        // MoveToPosition(speed,54.375,58.625,0); // 15
        // MoveToPosition(speed,54.375,8,0); // 16
        
        // MoveToPosition(speed,48,8,180); // 2
        // MoveToPosition(speed,0,22,0); // 3
        // MoveToPosition(speed,48,8,180); // 2
        // MoveToPosition(speed,0,22,0); // 3
        // MoveToPosition(speed,48,8,180); // 2
        // MoveToPosition(speed,0,22,0); // 3
        // MoveToPosition(speed,48,8,180); // 2
        // MoveToPosition(speed,55.5,56.75,0); // 9
        // MoveToPosition(speed,53,8,0); // 10
        // MoveToPosition(speed,53,15,0); // 11
        // MoveToPosition(speed,53,8,180); // 12
        // MoveToPosition(speed,0,28,0); // 13
        // MoveToPosition(speed,48,8,180); // 2
        // MoveToPosition(speed,0,28,0); // 3
        // MoveToPosition(speed,48,8,180); // 2
        // MoveToPosition(speed,0,30.75,0); // 3
        // MoveToPosition(speed,48,8,180); // 2
        Stop();
    }
    
    private void TestSequence()
    {
        MoveToPosition(speed,0,0,0,0,90,true,'N');
        // sleep(2000);
        // MoveToPosition(0.4,0,12,0);
        // sleep(2000);
        // s1.OdometryControl(0.2,12,0,0);
    }
 
    @Override
    public void runOpMode() 
    {
        dt.init(hardwareMap);
        cs.init(hardwareMap);
        am.init(hardwareMap);
        e1.init(hardwareMap);
        s1.init(hardwareMap);
        // s2.init(hardwareMap);
        // s3.init(hardwareMap);
        
        double volts = hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        double normalizedVolts = (1 - volts/14.5) + 0.7586;
        speed *= normalizedVolts;
       
        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                CommandSequence();
                //TestSequence();
            }
        }
    }
}

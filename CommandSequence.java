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
    
    // double output[] = new double[3];
    
    boolean moving = true;
    double speed = 0.6;
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
                            e1.move(speed,tgtE,'A',0);
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
                            am.rotate(tgtA,'A',0);
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
            telemetry.addData("position a",am.GetCurrentPosition());
            telemetry.addData("position e",e1.extension_1.getCurrentPosition());
            telemetry.addData("position claw",cs.GetClawPosition());
            telemetry.addData("position wrist",cs.GetWristState());
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
        
        //at 0.6 speed MoveToPosition 22 actually means MoveToPosition 30
        //at 0.6 speed MoveToPosition 10 actually means MoveToPosition 22
        
        // length = 15 inches
        // width = 15.25 inches
        // height = 15 inches
        // 211 degree ROM
        
        //32.5 = 40 - halfLength

        // first hook
        MoveToPosition(speed,0,48 - 2 * halfLength,0,600,70,false,'N');
        MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,false,'N'); 
        MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,true,'N');
        
        // // grab second specimen off wall
        // MoveToPosition(speed,48,15,0,0,211,true,'N');
        // MoveToPosition(speed,48,15,0,0,211,false,'N');

        // // second hook
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,600,76,false,'N'); 
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,false,'N'); 
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,true,'N');

        // // grab next sample
        // MoveToPosition(speed,30,42 - halfLength,0,0,211,true,'N');
        // MoveToPosition(speed,48 - halfWidth,48,0,0,211,true,'N');

        // // wait and then regrab specimen
        // MoveToPosition(speed,48 - halfWidth,15,0,0,111,true,'N');
        // sleep(500);
        // MoveToPosition(speed,48 - halfWidth,15,0,0,211,true,'N');
        // MoveToPosition(speed,48 - halfWidth,15,0,0,211,false,'N');

        // // third hook
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,600,76,false,'N'); 
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,false,'N'); 
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,true,'N');

        // // grab next sample
        // MoveToPosition(speed,30,42 - halfLength,0,0,211,true,'N');
        // MoveToPosition(speed,58 - halfWidth,48,0,0,211,true,'N');

        // // wait and then regrab specimen
        // MoveToPosition(speed,58 - halfWidth,15,0,0,111,true,'N');
        // sleep(500);
        // MoveToPosition(speed,58 - halfWidth,15,0,0,211,true,'N');
        // MoveToPosition(speed,58 - halfWidth,15,0,0,211,false,'N');

        // // fourth hook
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,600,76,false,'N'); 
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,false,'N'); 
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,true,'N');

        // // grab next sample
        // MoveToPosition(speed,30,42 - halfLength,0,0,211,true,'N');
        // MoveToPosition(speed,68 - halfWidth,48,0,0,211,true,'N');

        // // wait and then regrab specimen
        // MoveToPosition(speed,68 - halfWidth,15,0,0,111,true,'N');
        // sleep(500);
        // MoveToPosition(speed,68 - halfWidth,15,0,0,211,true,'N');
        // MoveToPosition(speed,68 - halfWidth,15,0,0,211,false,'N');

        // // fifth hook
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,600,76,false,'N'); 
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,false,'N'); 
        // MoveToPosition(speed,0,48 - 2 * halfLength,0,500,70,true,'N');

        // end sequence
        Stop();
    }
    
    private void TestSequence()
    {
        MoveToPosition(speed,0,20 - halfLength,0,0,0,false,'N');
        // sleep(2000);
        // MoveToPosition(speed,20 - halfWidth,20 - halfLength,0,0,0,false,'N'); 
        // sleep(2000);
        // MoveToPosition(speed,0,32 - halfLength,0,0,0,false,'N');
        
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
        double normalizedVolts = (1 - volts/13) + 0.7586;
        speed *= normalizedVolts;
       
        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                //CommandSequence();
                TestSequence();
            }
        }
    }
}


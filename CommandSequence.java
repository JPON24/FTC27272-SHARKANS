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
// import org.firstinspires.ftc.teamcode.PIDX;
// import org.firstinspires.ftc.teamcode.PIDH;
// import org.firstinspires.ftc.teamcode.PIDY;

@Autonomous
public class CommandSequence extends LinearOpMode
{
    ElapsedTime runtime = new ElapsedTime();
    
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    Extension_1 e1 = new Extension_1();
    OdometrySensor s1 = new OdometrySensor();
    
    // double output[] = new double[3];
    
    // PIDX s1 = new PIDX();
    // PIDY s2 = new PIDY();
    // PIDH s3 = new PIDH();

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
    
    private void MoveToPosition(double speed, double tgtX, double tgtY, double rot, int tgtE, int tgtA, boolean clawOpen, char wristMode)
    {
        if (speed != 0)
        {
            AddElement("n");
        }
         if (tgtX != 0)
        {
            AddElement("n");
        }
         if (tgtY != 0)
        {
            AddElement("n");
        }
         if (rot != 0)
        {
            AddElement("n");
        }
         if (tgtE != 0)
        {
            AddElement("n");
        }
         if (tgtA != 0)
        {
            AddElement("n");
        }
         if (clawOpen != false)
        {
            AddElement("n");
        }
         if (wristMode != 'D')
        {
            AddElement("n");
        }
            
        //s1.OdometryControl(speed,tgtX,tgtY,rot);
        
        // while (!s1.Completed() && !s2.Completed() && s3.Completed())
        while (!s1.AllBoolsCompleted())
        {
            s1.OdometryControl(speed,tgtX,tgtY,rot);
            
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
            telemetry.update();
            UpdateClaw();
            
            // if (runtime.milliseconds() > 5000)
            // {
            //     break;
            // }
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
    
    private void CommandSequence()
    {
        // push samples instead of grabbing for efficiency
        // push one hook one push one hook one for human player efficienccy
        
        //CURRENTLY 3 SECONDS TOO SLOW WITHOUT EVEN MANIPULATING
        
        MoveToPosition(speed,0,22,0); // 1
        // MoveToPosition(speed,0,25,180); // 1
        MoveToPosition(speed,48,8,180); // 2
        MoveToPosition(speed,0,28,0); // 3
        MoveToPosition(speed,39,18.75,0); // 4 
        MoveToPosition(speed,39,29,0); // 5 
        MoveToPosition(speed,39,8,180); // 6 
        MoveToPosition(speed,49,29,0); // 7
        MoveToPosition(speed,49,8,180); // 8
        MoveToPosition(speed,55.5,56.75,0); // 9
        MoveToPosition(speed,53,8,0); // 10
        MoveToPosition(speed,53,15,0); // 11
        MoveToPosition(speed,53,8,180); // 12
        MoveToPosition(speed,0,28,0); // 13
        MoveToPosition(speed,48,8,180); // 2
        MoveToPosition(speed,0,28,0); // 3
        MoveToPosition(speed,48,8,180); // 2
        MoveToPosition(speed,0,30.75,0); // 3
        MoveToPosition(speed,48,8,180); // 2
        Stop();
    }
    
    private void CommandSequenceV2()
    {
        MoveToPosition(speed,0,22,0); // 1
        MoveToPosition(speed,48,8,180); // 2
        MoveToPosition(speed,0,22,0); // 3
        
        //when adding new values subtract 3 inches for deceleration
        MoveToPosition(speed,28,18,0); // 5 
        MoveToPosition(speed,28,58.625,0); // 6 
        MoveToPosition(speed,39,58.625,0); // 7
        MoveToPosition(speed,39,8,0); // 8
        
        MoveToPosition(speed,38,18,0); // 9
        MoveToPosition(speed,38,58.625,0); // 10 
        MoveToPosition(speed,49,58.625,0); // 11
        MoveToPosition(speed,49,8,0); // 12
        
        MoveToPosition(speed,49,18,0); // 13 
        MoveToPosition(speed,49,58.625,0); // 14 
        MoveToPosition(speed,54.375,58.625,0); // 15
        MoveToPosition(speed,54.375,8,0); // 16
        
        MoveToPosition(speed,48,8,180); // 2
        MoveToPosition(speed,0,22,0); // 3
        MoveToPosition(speed,48,8,180); // 2
        MoveToPosition(speed,0,22,0); // 3
        MoveToPosition(speed,48,8,180); // 2
        MoveToPosition(speed,0,22,0); // 3
        MoveToPosition(speed,48,8,180); // 2
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
        MoveToPosition(0.4,0,12,180);
        sleep(2000);
        MoveToPosition(0.4,0,12,0);
        sleep(2000);
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
                // CommandSequence();
                CommandSequenceV2();
                // TestSequence();
            }
        }
    }
}

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

@Autonomous
public class CommandSequence extends LinearOpMode
{
    ElapsedTime runtime = new ElapsedTime();
    
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    Extension_1 e1 = new Extension_1();
    OdometrySensor s1 = new OdometrySensor();

    boolean moving = true;
    double speed = 0.5;
    int currentAction = 1;
    
    private void ContinueMovement()
    {
        moving = true;
    }
    
    private void MoveToPosition(double speed, double tgtX, double tgtY, double rot)
    {
        s1.OdometryControl(speed,tgtX,tgtY,rot);
        while (!s1.AllBoolsCompleted())
        {
            s1.OdometryControl(speed,tgtX,tgtY,rot);
            telemetry.addData("x",s1.GetPositionX());
            telemetry.addData("y",s1.GetPositionY());
            telemetry.addData("h",s1.GetImuReading());
            telemetry.addData("current action",currentAction);
            telemetry.addData("volts normalized",hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage());
            telemetry.addData("speed",speed);
            telemetry.addData("error x",s1.GetErrorX());
            telemetry.addData("error y",s1.GetErrorY());
            telemetry.addData("error h",s1.GetErrorH());
            telemetry.update();
            UpdateClaw();
            if (runtime.milliseconds() > 5000)
            {
                break;
            }
        }
        
        runtime.reset();
        currentAction += 1;
    }

    private void Stop()
    {
        dt.fieldOrientedTranslate(0,0,0);
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
        MoveToPosition(speed,0,25,0); // 1
        MoveToPosition(speed,48,3,180); // 2
        MoveToPosition(speed,0,28,0); // 3
        MoveToPosition(speed,39,18.75,0); // 4 breaks here
        MoveToPosition(speed,39,29,0); // 5 
        MoveToPosition(speed,39,3,180); // 6 
        MoveToPosition(speed,49,29,0); // 7
        MoveToPosition(speed,49,3,180); // 8
        MoveToPosition(speed,55.5,56.75,0); // 9
        MoveToPosition(speed,55.5,3,0); // 10
        MoveToPosition(speed,55.5,15,0); // 11
        MoveToPosition(speed,55.5,3,180); // 12
        MoveToPosition(speed,0,28,0); // 13
        MoveToPosition(speed,48,3,180); // 2
        MoveToPosition(speed,0,28,0); // 3
        MoveToPosition(speed,48,3,180); // 2
        MoveToPosition(speed,0,30.75,0); // 3
        MoveToPosition(speed,48,3,180); // 2
        Stop();
    }
    
    private void TestSequence()
    {
        // MoveToPosition(0.2,0,24,0);
        s1.OdometryControl(0.2,12,0,0);
        telemetry.addData("x",s1.GetPositionX());
        telemetry.addData("y",s1.GetPositionY());
        telemetry.addData("h",s1.GetImuReading());
        telemetry.addData("current action",currentAction);
        telemetry.addData("volts normalized",hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage());
        telemetry.addData("speed",speed);
        telemetry.addData("error x",s1.GetErrorX());
        telemetry.addData("error y",s1.GetErrorY());
        telemetry.addData("error h",s1.GetErrorH());
        telemetry.update();
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
        double normalizedVolts = (1 - volts/14.5) + 0.7586;
        speed *= normalizedVolts;
       
        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                // CommandSequence();
                TestSequence();
            }
        }
    }
}

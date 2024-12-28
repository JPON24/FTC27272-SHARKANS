package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.ClawServo;
import org.firstinspires.ftc.teamcode.Extension_1;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;
import org.firstinspires.ftc.teamcode.Sensor1;

@Autonomous

public class AutonomousRight extends LinearOpMode
{
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    Extension_1 e1 = new Extension_1();
    Sensor1 s1 = new Sensor1();

    double collisionDetectionRadius = 6.5;
    boolean moving = true;
    boolean grabbing = true;
    int id =0;
    
    private void ContinueMovement()
    {
        moving = true;
    }
    
    private void MoveToPosition(double speed, double tgtX, double tgtY, double rot)
    {
        //if (action.GetActionId()!=id){return;}
        s1.OdometryControl(speed,tgtX,tgtY,rot);
    }
    private void MoveForward(double speed)
    {
        dt.translate(0,-speed,0);
        UpdateClaw();
    }
    private void MoveBackward(double speed)
    {
        dt.translate(0,speed,0);
        UpdateClaw();
    }
    
    private void MoveLeft(double speed)
    {
        dt.translate(-speed,0,0);
        UpdateClaw();
    }
    private void MoveRight(double speed)
    {
        dt.translate(speed,0,0);
        UpdateClaw();  
    }
    
    private void Stop()
    {
        dt.translate(0,0,0);
        UpdateClaw();
    }
    
    private void LeftRotate()
    {
        dt.translate(0,0,-1);
         UpdateClaw();
    }
    private void RightRotate()
    {
        dt.translate(0,0,1);
        UpdateClaw();
    }
    
    private void OpenClaw()
    {
        cs.clawMove(false);
        UpdateClaw();
    }
    private void CloseClaw()
    {
        cs.clawMove(true);
        UpdateClaw();
    }
    
    private void Extend(double power, int tgt)
    {
        UpdateClaw();
        id++;
        //if (action.GetActionId() != id) {return;}
        e1.move(power,tgt,"A",id);
    }
    private void Retract(double power, int tgt)
    {
        UpdateClaw();
        id++;
        //if(action.GetActionId()!=id){return;}
        e1.move(-power,tgt,"A",id);
    }
    
    private void RotateArm(double rotation)
    {
        UpdateClaw();
        id++;
        am.rotate(rotation,"A",id);
    }
    
    private void ActivateGrab()
    {
        grabbing = true;
    }
    private void DeactivateGrab()
    {
        grabbing = false;
    }
    
    private void UpdateClaw()
    {
        cs.update();
    }
    
    private void CommandSequence()
    {
        MoveToPosition(0.8,0,30.75,0); // 1
        sleep(1600);
        MoveToPosition(0.8,48,3,180); // 2
        sleep(1600);
        MoveToPosition(0.8,0,30.75,0); // 3
        sleep(1600);
        MoveToPosition(0.8,39,18.75,0); // 4
        sleep(1600);
        MoveToPosition(0.8,39,29,0); // 5
        sleep(1600);
        MoveToPosition(0.8,39,3,180); // 6
        sleep(1600);
        MoveToPosition(0.8,49,29,0); // 7
        sleep(1600);
        MoveToPosition(0.8,49,3,180); // 8
        sleep(1600);
        MoveToPosition(0.8,55.5,56.75,0); // 9
        sleep(1600);
        MoveToPosition(0.8,55.5,3,0); // 10
        sleep(1600);
        MoveToPosition(0.8,55.5,15,0); // 11
        sleep(1600);
        MoveToPosition(0.8,55.5,3,180); // 12
        sleep(1600);
        MoveToPosition(0.8,0,30.75,0); // 13
        sleep(1600);
        MoveToPosition(0.8,48,3,180); // 2
        sleep(1600);
        MoveToPosition(0.8,0,30.75,0); // 3
        sleep(1600);
        MoveToPosition(0.8,48,3,180); // 2
        sleep(1600);
        MoveToPosition(0.8,0,30.75,0); // 3
        sleep(1600);
        MoveToPosition(0.8,48,3,180); // 2
        sleep(1600);
        Stop();
    }
 
    @Override
    public void runOpMode() 
    {
        dt.init(hardwareMap);
        cs.init(hardwareMap);
        am.init(hardwareMap);
        e1.init(hardwareMap);
       
        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                CommandSequence();
            }
        }
    }
}

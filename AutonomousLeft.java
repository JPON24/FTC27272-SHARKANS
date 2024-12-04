package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.ClawServo;
import org.firstinspires.ftc.teamcode.Extension_1;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;

@Autonomous

public class AutonomousLeft extends LinearOpMode
{
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    Extension_1 e1 = new Extension_1();

    double collisionDetectionRadius = 6.5;
    boolean moving = true;
    boolean grabbing = true;
    
    private void ContinueMovement()
    {
        moving = true;
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
    
    private void Extend(double power)
    {
        e1.move(power);
        UpdateClaw();
    }
    private void Retract(double power)
    {
        e1.move(-power);
        UpdateClaw();
    }
    
    private void RotateArm(double rotation)
    {
        am.rotate(rotation,"A");
        UpdateClaw();
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
        ActivateGrab();
        CloseClaw();
        sleep(2000);
        RotateArm(90);
        Extend(1);//simultaneous
        sleep(2500);
        Extend(1);//simultaneous
        sleep(2000);
        MoveForward(0.8);
        sleep(550);
        Stop();
        sleep(1000);
        RotateArm(70);
        sleep(1000);
        RotateArm(60);
        sleep(1000);
        Retract(0.5);// latch
        sleep(300);
        Retract(0);
        OpenClaw();
        DeactivateGrab();
        sleep(500);
        MoveBackward(0.2);
        sleep(1300);
        Stop();
        sleep(400);
        Retract(1);//simultaneous
        sleep(200);
        Retract(0);
        RotateArm(30);//simultaneous
        MoveLeft(0.35);//simultaneous
        sleep(2200);
        Stop();
        sleep(400);
        MoveForward(0.8);
        sleep(500);
        Stop();
        sleep(500);
        MoveLeft(0.6);
        sleep(350);
        Stop();
        sleep(400);
        MoveBackward(0.5);//brief
        sleep(3000);
        Stop();
        moving = false;
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

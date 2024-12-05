package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
//import org.firstinspires.ftc.robotcore.external.navigation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.ClawServo;
import org.firstinspires.ftc.teamcode.Extension_1;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;

@Autonomous

public class AutonomousRight extends LinearOpMode
{
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    Extension_1 e1 = new Extension_1();
    
    IMU imu;

    double collisionDetectionRadius = 6.5;
    boolean moving = true;
    boolean grabbing = true;
    
    private void ContinueMovement()
    {
        moving = true;
    }
    private void MoveForward(double speed, double rot)
    {
        double rotationRequired = GetRequiredCorrection(rot);
        dt.translate(0,-speed,rotationRequired * speed);
        UpdateClaw();
    }
    private void MoveBackward(double speed,double rot)
    {
        double rotationRequired = GetRequiredCorrection(rot);
        dt.translate(0,speed,rotationRequired * speed);
        UpdateClaw();
    }
    
    private void MoveLeft(double speed, double rot)
    {
        double rotationRequired = GetRequiredCorrection(rot);
        dt.translate(-speed,0,rotationRequired * speed);
        UpdateClaw();
    }
    private void MoveRight(double speed, double rot)
    {
        double rotationRequired = GetRequiredCorrection(rot);
        dt.translate(speed,0,rotationRequired * speed);
        UpdateClaw();  
    }
    
    private void Stop()
    {
        dt.translate(0,0,0);
        UpdateClaw();
    }
    
    private void LeftRotate(double speed)
    {
        dt.translate(0,0,-speed);
         UpdateClaw();
    }
    private void RightRotate(double speed)
    {
        dt.translate(0,0,speed);
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

    private double GetRequiredCorrection(double tgt)
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);

        int lenience = 5;
        if (yaw > tgt - lenience && yaw < tgt + lenience)
        {
            return 0;
        }
        else if (yaw < tgt)
        {
            return -1;
        }
        else if (yaw > tgt)
        {
            return 1;
        }
    }

    private void IMU_RotationControl(int tgt, double speed, String dir)
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);
        if (dir == "R")
        {
            if (!(yaw > tgt - 5 && yaw < tgt + 5))
            {
                RightRotate(speed);
            }
            else
            {
                Stop();
            }
        }
        else if (dir == "L")
        {
            if (!(yaw > tgt - 5 && yaw < tgt + 5))
            {
                LeftRotate(speed);
            }
            else
            {
                RightRotate(speed);
                sleep(50);
                Stop();
                moving = false;
            }
        }
    }
    
    private void CommandSequence()
    {
        ActivateGrab();
        CloseClaw();
        sleep(2000);
        RotateArm(90);
        Extend(1.8);//simultaneous
        sleep(2500);
        MoveForward(0.8,0);
        sleep(550);
        Stop();
        sleep(1000);
        RotateArm(70);
        sleep(1000);
        RotateArm(60);
        sleep(1000);
        Retract(0.5);// latch
        sleep(400);
        Retract(0);
        OpenClaw();
        DeactivateGrab();
        sleep(500);
        MoveBackward(0.9,0);
        sleep(290);
        Stop();
        sleep(400);
        Retract(1);//simultaneous
        sleep(200);
        Retract(0);
        RotateArm(30);//simultaneous
        MoveRight(0.8,0);//simultaneous
        sleep(800);
        IMU_RotationControl(0,1,"R");
        sleep(200);
        MoveForward(0.8,0);
        sleep(750);
        Stop();
        sleep(500);
        MoveRight(0.6,0);
        sleep(350);
        Stop();
        sleep(400);
        MoveBackward(0.8,0);// first sample
        
        sleep(900);
        Stop();
        sleep(300);
        MoveLeft(0.5,0);
        sleep(150);
        Stop();
        sleep(100);
        MoveForward(0.8,0);
        sleep(900);
        Stop();
        sleep(200);
        MoveRight(0.6,0);
        sleep(500);
        Stop();
        sleep(100);
        MoveBackward(0.8,0);
        
        sleep(900);
        Stop();
        sleep(300);
        MoveLeft(0.5,0);
        sleep(150);
        Stop();
        sleep(100);
        MoveForward(0.8,0);
        sleep(900);
        IMU_RotationControl(0,0.8,"R");
        sleep(200);
        MoveRight(0.6,0);
        sleep(500);
        Stop();
        sleep(100);
        MoveBackward(0.8,0);
        sleep(1000);
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
        
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot robotOrientationInit = new RevHubOrientationOnRobot(logoDir,usbDir);
        imu.initialize(new IMU.Parameters(robotOrientationInit));
        imu.resetYaw();
        
        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                CommandSequence();
                /* 
                IMU_RotationControl(85,1,"L");
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                double yaw = orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("yaw",yaw);
                telemetry.update();*/
            }
        }
    }
}

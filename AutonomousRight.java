package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
//import org.firstinspires.ftc.robotcore.external.navigation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    DcMotor extension;
    
    private void ContinueMovement()
    {
        moving = true;
    }
    private void MoveForward(double speed, double rot)
    {
        double rotationRequired = GetRequiredCorrection(rot);
        dt.fieldOrientedTranslate(0,speed,rotationRequired * speed);
        UpdateClaw();
    }
    private void MoveBackward(double speed,double rot)
    {
        double rotationRequired = GetRequiredCorrection(rot);
        dt.fieldOrientedTranslate(0,-speed,rotationRequired * speed);
        UpdateClaw();
    }
    
    private void MoveLeft(double speed, double rot)
    {
        double rotationRequired = GetRequiredCorrection(rot);
        dt.fieldOrientedTranslate(-speed,0,rotationRequired * speed);
        UpdateClaw();
    }
    private void MoveRight(double speed, double rot)
    {
        double rotationRequired = GetRequiredCorrection(rot);
        dt.fieldOrientedTranslate(speed,0,rotationRequired * speed);
        UpdateClaw();  
    }
    
    private void Stop()
    {
        dt.fieldOrientedTranslate(0,0,0);
        UpdateClaw();
    }
    
    private void LeftRotate(double speed)
    {
        dt.fieldOrientedTranslate(0,0,-speed);
        UpdateClaw();
    }
    private void RightRotate(double speed)
    {
        dt.fieldOrientedTranslate(0,0,speed);
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
        e1.move(power,tgt,"A");
        UpdateClaw();
    }
    private void Retract(double power, int tgt)
    {
        e1.move(-power,tgt,"A");
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
        //cs.update();
    }

    private double GetRequiredCorrection(double tgt)
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);

        int lenience = 1; //lenience in degrees
        if (yaw > tgt - lenience && yaw < tgt + lenience)
        {
            return 0;
        }
        else if (yaw < tgt - lenience)
        {
            return -0.05; //turn speed
        }
        else if (yaw > tgt + lenience)
        {
            return 0.05; //turn speed
        }
        return 0;
    }

    private void IMU_RotationControl(int tgt, double speed, String dir)
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);
        double lenience = 2;
        if (dir == "R")
        {
            if (!(yaw > tgt - lenience && yaw < tgt + lenience))
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
            if (!(yaw > tgt - lenience && yaw < tgt + lenience))
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
        ActivateGrab(); //grab sample
        CloseClaw(); //grab sample
        sleep(1000);
        Extend(1,150);
        sleep(1000);
        RotateArm(90);//lift sample
        Extend(1, 700);//try to reach high chamber
        sleep(2500);
        MoveForward(0.8,0);//go to submersible
        sleep(575);
        Stop();//reset wheels 
        sleep(1000);
        RotateArm(70);//start to clip sample
        sleep(1000);
        Retract(1, 600);// latch
        sleep(400);
        RotateArm(60);//pull 
        sleep(1000);
        Retract(1,450);//secure latch
        sleep(1000);
        OpenClaw();//release specimen
        DeactivateGrab();//release specime
        sleep(500);
        
        MoveBackward(0.9,0);//prepare to get samples
        sleep(290);
        Stop();//reset wheels
        sleep(400);
        Retract(1,150);
        RotateArm(30);//reset arm 
        MoveRight(0.8,0);//move to samples
        sleep(800);
        MoveForward(0.8,0);//prepare to push samples
        sleep(900);
        Stop();//reset wheels
        sleep(500);
        MoveRight(0.8,0);//line up with samples
        sleep(250);
        Stop();//reset wheels
        sleep(400);
        MoveBackward(0.8,0);// first sample
        sleep(1000);
        Stop();//reset wheels
        sleep(100);
        MoveForward(0.8,0);//go to samples
        sleep(850);
        Stop();//reset wheels
        sleep(200);
        MoveRight(0.6,0);//line up with sample
        sleep(400);
        Stop();//reset wheels
        sleep(100);
        MoveBackward(0.8,0);//second sample
        
        sleep(1000);
        Stop();//reset wheels
        sleep(300);
        MoveForward(0.8,0);//go to sample
        sleep(900);
        Stop();
        sleep(200);
        MoveRight(0.6,0);//line up with sample
        sleep(300);
        Stop();//reset wheels
        sleep(100);
        MoveBackward(0.8,0);//sample 3 and park
        sleep(900);
        Stop();//reset wheels
        moving = false;//stop driving, finish auto
    }
 
    @Override
    public void runOpMode() 
    {
        dt.init(hardwareMap);
        cs.init(hardwareMap);
        am.init(hardwareMap);
        e1.init(hardwareMap);
        
        imu = hardwareMap.get(IMU.class, "imu");
        extension = hardwareMap.get(DcMotor.class, "extension_1");
        
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
                telemetry.addData("extension pos", extension.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

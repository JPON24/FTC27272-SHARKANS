/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.IMU;
//import org.firstinspires.ftc.robotcore.external.navigation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

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
    
    IMU imu;

    double collisionDetectionRadius = 6.5;
    boolean moving = true;
    boolean grabbing = true;
    
    int id = 0;

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
        cs.update();
    }
    
    private void MoveToPosition(double tgt, char dir)
    {
        s1.moveToPositionR(tgt,dir,id);
        id++;
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
        Extend(1, 750);//try to reach high chamber
        sleep(2500);
        MoveForward(0.55,0);//go to submersible
        //MoveToPosition(30,"F");
        sleep(575);
        Stop();//reset wheels 
        sleep(1000);
        RotateArm(70);//start to clip sample
        sleep(400);
        RotateArm(60);//pull 
        sleep(1000);
        RotateArm(50);//pull
        sleep(500);
        Retract(1,450);//secure latch
        sleep(1500);
        OpenClaw();//release specimen
        DeactivateGrab();//release specimen
        sleep(500);
        
        MoveBackward(0.5,0);//prepare to get samples\
        //MoveToPosition(14,"B");
        sleep(290);
        Stop();//reset wheels
        sleep(400);
        Retract(1,150);
        RotateArm(30);//reset arm 
        MoveRight(0.5,0);//move to samples
        //MoveToPosition(27,"R");
        sleep(1000);
        MoveForward(0.5,0);//prepare to push samples
        //MoveToPosition(40,"F");
        sleep(800);
        Stop();//reset wheels
        sleep(500);
        MoveRight(0.5,0);//line up with samples
        //MoveToPosition(8,"R");
        sleep(400);
        Stop();//reset wheels
        sleep(400);
        MoveBackward(0.5,0);// first sample
        //MoveToPosition(48,"B");
        sleep(1250);
        Stop();//reset wheels
        sleep(100);
        MoveForward(0.5,0);//go to samples
        //MoveToPosition(48,"F");
        sleep(1200);
        Stop();//reset wheels
        sleep(200);
        MoveRight(0.6,0);//line up with sample
        //MoveToPosition(8,"R");
        sleep(200);
        Stop();//reset wheels
        sleep(100);
        MoveBackward(0.5,0);//second sample
        //MoveToPosition(48,"B");
        sleep(1250);
        Stop();
        moving = false;
        id = 0;
    }
 
    @Override
    public void runOpMode() 
    {
        dt.init(hardwareMap);
        cs.init(hardwareMap);
        am.init(hardwareMap);
        e1.init(hardwareMap);
        s1.init(hardwareMap);
        
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
                /*telemetry.addData("extension pos", extension.getCurrentPosition());
                telemetry.update()
            }
        }
    }
}*/

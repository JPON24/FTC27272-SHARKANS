package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.ClawServo;
import org.firstinspires.ftc.teamcode.Extension_1;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous

public class AutonomousFallback extends LinearOpMode
{
    Drivetrain dt = new Drivetrain();
    boolean moving = true;
    
    private void MoveRight(double speed)
    {
        dt.translate(speed,0,0);
    }
    private void Stop()
    {
        dt.translate(0,0,0);
    }
    
    @Override
    public void runOpMode() 
    {
        dt.init(hardwareMap);
        
        waitForStart();
        while(opModeIsActive())
        {
            if (moving)
            {
                MoveRight(0.1);
                sleep(10000);
            }
        }
    }
}

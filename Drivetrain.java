package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain{
    private double speedScalar = 1; // used to control speed with bumpers
    
    //motor references
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    IMU imu;
    
    public void init(HardwareMap hwMap)
    {
        // intialize motors
        frontLeft = hwMap.get(DcMotor.class, "FrontLeft");
        frontRight = hwMap.get(DcMotor.class, "FrontRight");
        backLeft = hwMap.get(DcMotor.class, "BackLeft");
        backRight = hwMap.get(DcMotor.class, "BackRight");

        //intialize imu
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot robotOrientationInit = new RevHubOrientationOnRobot(logoDir,usbDir);
        imu.initialize(new IMU.Parameters(robotOrientationInit));
        imu.resetYaw();

        //set their run modes to run without encoder, meaning they will not be using setTargetPos
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void fieldOrientedTranslate(double targetPowerX, double targetPowerY, double rotation)
    {
        //get rotation from imu
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);

        double polarControllerInput = Math.atan2(targetPowerY,targetPowerX) * 180/PI; // get controller stick dir

        double theta = (180 - polarControllerInput) + yaw; //convert to -180<=x<=180 and add offset

        //scale based on theta
        double flScalar = sin(theta);
        double frScalar = cos(theta);
        double blScalar = cos(theta);
        double brScalar = sin(theta);

        //apply scalars + basic code
        double frontLeftPower = (-rotation + targetPowerY -targetPowerX) * flScalar;
        double frontRightPower = (-rotation -targetPowerY -targetPowerX) * frScalar;
        double backLeftPower = (-rotation + targetPowerY + targetPowerX) * blScalar;
        double backRightPower = (-rotation -targetPowerY + targetPowerX) * brScalar;

        //set speed * speed scalar
        frontLeft.setPower(frontLeftPower * speedScalar);
        frontRight.setPower(frontRightPower * speedScalar);
        backLeft.setPower(backLeftPower * speedScalar);
        backRight.setPower(backRightPower * speedScalar);
    }
    
    public void translate(double targetPowerX, double targetPowerY, double rotation)
    {
        // inputs target power on x and y, outputs proper power distribution
        double frontLeftPower = -rotation + targetPowerY -targetPowerX;
        double frontRightPower = -rotation -targetPowerY -targetPowerX;
        double backLeftPower = -rotation + targetPowerY + targetPowerX;
        double backRightPower = -rotation -targetPowerY + targetPowerX;

        //sets power of each motor
        frontLeft.setPower(frontLeftPower * speedScalar);
        frontRight.setPower(frontRightPower * speedScalar);
        backLeft.setPower(backLeftPower * speedScalar);
        backRight.setPower(backRightPower * speedScalar);
    }
    
    public void SetSpeedScalar(double change)
    {
        //sets the speed scalar of the motors to a specified value in starter bot
        speedScalar = change;
    }
}

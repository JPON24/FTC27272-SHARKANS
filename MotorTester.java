package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@TeleOp

public class MotorTester extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor amL = null;
    private DcMotor amR = null;
    private Servo claw = null;
    private Servo wrist = null;
    private SparkFunOTOS odometry;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        amL = hardwareMap.get(DcMotor.class, "armLiftL");
        amR = hardwareMap.get(DcMotor.class, "armLiftR");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        
        odometry = hardwareMap.get(SparkFunOTOS.class,"otos");
        odometry.resetTracking();
        odometry.begin();
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.setLinearScalar(40/41);
        odometry.setAngularScalar(1);
        odometry.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte)0x0D));
        odometry.resetTracking();
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        amR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        amL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        amR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        
        
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.x)
            {
                frontLeft.setPower(1); //frontright
            }
            else if(gamepad1.a)
            {
                frontRight.setPower(1); //backright
            }
            else if(gamepad1.y)
            {
                backLeft.setPower(1); //frontleft
            }
            else if(gamepad1.b)
            {
                backRight.setPower(1); //backleft
            }
            if (gamepad1.left_bumper)
            {
                claw.setPosition(0.1);
            }
            else if (gamepad1.right_bumper)
            {
                claw.setPosition(0.3);   
            }
            
            if (gamepad2.x)
            {
                amL.setPower(-0.1);
                amR.setPower(-0.1);
            }
            else if (gamepad2.a)
            {
                amL.setPower(0.1);
                amR.setPower(0.1);
            }
            else
            {
                amL.setPower(0);
                amR.setPower(0);
            }

            if (gamepad2.y)
            {
                wrist.setPosition(0.5);
                // wrist.setPosition(0.3);
            }
            else if (gamepad2.b)
            {
                wrist.setPosition(1);
                // wrist.setPosition(0);
            }
            else
            {
                wrist.setPosition(0);
                // wrist.setPower(0);
            }
            
            claw.setPosition(0);
            
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            
            telemetry.addData("frontLeftPosition", backLeft.getCurrentPosition());
            telemetry.addData("odometryx", odometry.getPosition().x);
            telemetry.addData("odometryy", odometry.getPosition().y);
            telemetry.addData("odometryh", odometry.getPosition().h);
            telemetry.addData("armLPosition", amL.getCurrentPosition());
            telemetry.addData("armRPosition", amR.getCurrentPosition());
            telemetry.addData("wrist position", wrist.getPosition());
            telemetry.update();
        }
    }
}

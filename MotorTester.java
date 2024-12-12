package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;

@TeleOp

public class MotorTester extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor lift = null;
    private Servo claw = null;
    private MaxSonarI2CXL d;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        lift = hardwareMap.get(DcMotor.class, "armLiftMotor");
        claw = hardwareMap.get(Servo.class, "claw");
        //rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"Sensor1");
        d = hardwareMap.get(MaxSonarI2CXL.class, "Sensor1");
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
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
            
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            
            telemetry.addData("frontLeftPosition", backLeft.getCurrentPosition());
            telemetry.addData("liftPosition", lift.getCurrentPosition());
            //telemetry.addData("sensor reading",rangeSensor.rawUltrasonic());
            telemetry.addData("sensor reading", d.getDistanceSync(500,DistanceUnit.INCH));
            telemetry.update();
        }
    }
}

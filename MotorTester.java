package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
 import com.qualcomm.robotcore.hardware.CRServo;
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

    private DcMotor extend = null;

    private Servo claw = null;
    private Servo wristR = null;
    private Servo wristL = null;
    private Servo extendPitch = null;
    private Servo extendRoll = null;
    private Servo extendClaw = null;

    private SparkFunOTOS odometry;

//    private Limelight limelight = new Limelight();

    @Override
    public void runOpMode() {
//        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
//        backRight = hardwareMap.get(DcMotor.class, "BackRight");
//
        odometry = hardwareMap.get(SparkFunOTOS.class,"otos");
//        wristR = hardwareMap.get(Servo.class, "wristR");
//        wristL = hardwareMap.get(Servo.class, "wristL");
//        claw = hardwareMap.get(Servo.class, "claw");
        extend = hardwareMap.get(DcMotor.class, "extend");
        extendPitch = hardwareMap.get(Servo.class, "extendPitch");
        extendRoll = hardwareMap.get(Servo.class, "extendRoll");
        extendClaw = hardwareMap.get(Servo.class, "extendClaw");

//        limelight.init(hardwareMap);

        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.calibrateImu();
        odometry.setLinearScalar(1);
        odometry.setAngularScalar(1);
        odometry.setOffset(new SparkFunOTOS.Pose2D(0.4375,3.625,0));
        odometry.resetTracking();
        odometry.begin();
        
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        amL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        amR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        amL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        amR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.FORWARD);
        
        
        waitForStart();
        while (opModeIsActive()) {
//            extendPitch.setPosition(0);
//            wristR.setPosition(0);
//            wristL.setPosition(0);
//            claw.setPosition(1);
//            if(gamepad1.x)
//            {
//                frontLeft.setPower(1); //frontright
//            }
//            else if(gamepad1.a)
//            {
//                frontRight.setPower(1); //backright
//            }
//            else if(gamepad1.y)
//            {
//                backLeft.setPower(1); //frontleft
//            }
//            else if(gamepad1.b)
//            {
//                backRight.setPower(1); //backleft
//            }
//            if (gamepad1.left_bumper)
//            {
//                claw.setPosition(0.1);
//            }
//            else if (gamepad1.right_bumper)
//            {
//                claw.setPosition(0.3);
//            }
//
            if (gamepad2.x)
            {
                extend.setPower(-0.6);
            }
            else if (gamepad2.a)
            {
                extend.setPower(0.6);
            }
            else
            {
                extend.setPower(0);
            }

            if (gamepad2.y)
            {
//                extendRoll.setPosition(0);
//                extendPitch.setPosition(0);
                extendClaw.setPosition(0);
            }
            else if (gamepad2.b)
            {
//                extendRoll.setPosition(1);
//                extendPitch.setPosition(1);
                extendClaw.setPosition(0.4);
            }

//            if (gamepad2.x)
//            {
//                amL.setPower(-0.1);
//                amR.setPower(-0.1);
//            }
//            else if (gamepad2.a)
//            {
//                amL.setPower(0.1);
//                amR.setPower(0.1);
//            }
//            else
//            {
//                amL.setPower(0);
//                amR.setPower(0);
//            }
//
//            if (gamepad2.y)
//            {
//                wrist.setPosition(0.5);
//                // wrist.setPosition(0.3);
//            }
//            else if (gamepad2.b)
//            {
//                wrist.setPosition(1);
//                // wrist.setPosition(0);
//            }
//            else
//            {
//                wrist.setPosition(0);
//                // wrist.setPower(0);
//            }
//
//            claw.setPosition(0);
//
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//            backLeft.setPower(0);
//            backRight.setPower(0);
//
//            SparkFunOTOS.Pose2D position = limelight.GetLimelightData(true, odometry.getPosition().h);
//            telemetry.addData("frontLeftPosition", backLeft.getCurrentPosition());
//            telemetry.addData("odometryx", odometry.getPosition().x);
//            telemetry.addData("odometryy", odometry.getPosition().y);
//            telemetry.addData("odometryh", odometry.getPosition().h);
            telemetry.addData("extension", extend.getCurrentPosition());
//            telemetry.addData("x limelight localization", 'position.x);
//            telemetry.addData("y limelight localization", position.y);

//            telemetry.addData("armLPosition", amL.getCurrentPosition());
//            telemetry.addData("armRPosition", amR.getCurrentPosition());
//            telemetry.addData("wrist position", wrist.getPosition());
            telemetry.update();
        }
    }
}

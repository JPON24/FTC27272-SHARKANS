package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.ArmLiftMotor;
import org.firstinspires.ftc.teamcode.ClawServo;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Extension_1;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class StarterBot extends LinearOpMode{
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    Extension_1 e1 = new Extension_1();
    //private DcMotor LiftMotor = null;
    private DcMotor extension = null;
    private Servo claw = null;
    IMU imu;

    boolean fieldOriented = false;
    
    @Override
    public void runOpMode()
    {
        
        //LiftMotor = hardwareMap.get(DcMotor.class, "armLiftMotor");
        extension = hardwareMap.get(DcMotor.class, "extension_1");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");
        
        boolean canShift = true;
        double currentSpeed = 1.0;
        double speedInterval = 0.4;
        
        boolean canShiftArm = true;
        double currentArmSpeed = 1.0;
        double armSpeedInterval = 0.8;
            
        dt.init(hardwareMap);
        cs.init(hardwareMap);
        am.init(hardwareMap);
        e1.init(hardwareMap);
        waitForStart();
        
        while(opModeIsActive())
        {
            double targetPowerX = gamepad1.left_stick_x;
            double targetPowerY = -gamepad1.left_stick_y;
            double targetRotation = gamepad1.right_stick_x;
            
            boolean leftBumperPressed = gamepad1.left_bumper;
            boolean rightBumperPressed = gamepad1.right_bumper;
            boolean dpadUp = gamepad1.dpad_up;
            boolean aButtonPressed1 = gamepad1.a;
            boolean xButtonPressed = gamepad1.x;
            
            //arm
            boolean xButtonPressed = gamepad2.x;
            boolean aButtonPressed = gamepad2.a;
            double armLiftInput = -gamepad2.left_stick_y;
            double armExtendInput = -gamepad2.right_stick_y;
            
            boolean dpadUp2 = gamepad2.dpad_up;
            boolean dpadLeft2 = gamepad2.dpad_left;
            boolean leftBumperPressed2 = gamepad2.left_bumper;
            boolean rightBumperPressed2 = gamepad2.right_bumper;
            
            double leftTrigger2 = gamepad2.left_trigger;
            double rightTrigger2 = gamepad2.right_trigger;
            
            //claw
            if (xButtonPressed == true)
            {
                cs.clawMove(false);
            }
            else if (aButtonPressed == true)
            {
                cs.clawMove(true);
            }
            
            cs.update();
            
            // drive
            if (leftBumperPressed)
            {
                if (canShift && currentSpeed - speedInterval > 0)
                {
                    currentSpeed -= speedInterval;
                    dt.SetSpeedScalar(currentSpeed);
                    canShift = false;
                }
            }
            else if (rightBumperPressed)
            {
                if (canShift && currentSpeed + speedInterval <= 1)
                {
                    currentSpeed += speedInterval;
                    dt.SetSpeedScalar(currentSpeed); 
                    canShift = false;
                }  
            }
            else
            {
                canShift = true;
            }
            
            if (dpadUp)
            {
                imu.resetYaw();
            }
            
            //arm
            if (leftBumperPressed2)
            {
                if (canShiftArm && currentArmSpeed - armSpeedInterval > 0)
                {
                    currentArmSpeed -= armSpeedInterval;
                    am.SetSpeed(currentArmSpeed);
                    canShiftArm = false;
                }
            }
            else if (rightBumperPressed2)
            {
                if (canShiftArm && currentArmSpeed + armSpeedInterval <=1)
                {
                    currentArmSpeed += armSpeedInterval;
                    am.SetSpeed(currentArmSpeed);
                    canShiftArm = false;
                }
            }
            else
            {
                canShiftArm = true;
            }
            
            if(dpadUp2)//reset arm pos
            {
                am.ResetEncoders();
            }
            else if (dpadLeft2)//fix extension pos
            {
                e1.ResetEncoders();
            }

            if (aButtonPressed1)
            {
                fieldOriented = true;
            }
            else if (xButtonPressed1)
            {
                fieldOriented = false;
            }
            if (fieldOriented)
            {
                dt.fieldOrientedTranslate(targetPowerX,targetPowerY,targetRotation);
            }
            else
            {
                dt.translate(targetPowerX,targetPowerY,targetRotation);
            }
            
            if (rightTrigger2 > 0.8)
            {
                armLiftInput = 1;
                armExtendInput = 1;
            }
            else if (leftTrigger2 > 0.8)
            {
                armLiftInput = -1;
                armExtendInput = -1;
            }
            
            am.rotate(armLiftInput, "T");
            if (armExtendInput > 0.1)
            {
                e1.move(armExtendInput,775, "T");
            }
            else if (armExtendInput < -0.1)
            {
                e1.move(armExtendInput,25,"T");
            }
            else
            {
                e1.move(0,extension.getCurrentPosition(),"T");
            }
            /*
            telemetry.addData("extensionDistance", extension.getCurrentPosition());
            telemetry.addData("clawPosition", claw.getPosition());
            telemetry.addData("x",targetPowerX);
            telemetry.addData("y",targetPowerY);
            telemetry.addData("rot",targetRotation);
            telemetry.update();*/
        }
    }
}

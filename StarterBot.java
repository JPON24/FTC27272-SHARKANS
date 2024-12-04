package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    //private DcMotor extension = null;
    private Servo claw = null;
    
    @Override
    public void runOpMode()
    {
        
        //LiftMotor = hardwareMap.get(DcMotor.class, "armLiftMotor");
        //extension = hardwareMap.get(DcMotor.class, "extension_1");
        claw = hardwareMap.get(Servo.class, "claw");
        
        boolean canShift = true;
        double currentSpeed = 1.0;
        double speedInterval = 0.4;
            
        dt.init(hardwareMap);
        cs.init(hardwareMap);
        am.init(hardwareMap);
        e1.init(hardwareMap);
        waitForStart();
        
        while(opModeIsActive())
        {
            //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //drivetrain
            double targetPowerX = gamepad1.left_stick_x;
            double targetPowerY = gamepad1.left_stick_y;
            double targetRotation = gamepad1.right_stick_x;
            
            boolean leftBumperPressed = gamepad1.left_bumper;
            boolean rightBumperPressed = gamepad1.right_bumper;
            
            //arm
            boolean xButtonPressed = gamepad2.x;
            boolean aButtonPressed = gamepad2.a;
            double armLiftInput = -gamepad2.left_stick_y;
            double armExtendInput = -gamepad2.right_stick_y;
            if (xButtonPressed == true)
            {
                cs.clawMove(false);
            }
            else if (aButtonPressed == true)
            {
                cs.clawMove(true);
            }
            
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
            
            dt.translate(targetPowerX,targetPowerY,targetRotation);
            am.rotate(armLiftInput, "T");
            e1.move(armExtendInput);
            cs.update();
            //telemetry.addData("currentPosition", LiftMotor.getCurrentPosition());
            //telemetry.addData("motorPower", LiftMotor.getPower());
            //telemetry.addData("extensionDistance", extension.getCurrentPosition());
            telemetry.addData("clawPosition", claw.getPosition());
            telemetry.update();
        }
    }
}

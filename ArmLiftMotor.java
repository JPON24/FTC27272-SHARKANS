package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class ArmLiftMotor {
    
    private DcMotor armLift = null;
    private DcMotor extension = null;
    private double currentRotation = 0;
        
    double rpm = 84;
    double gearRatio = 5;
    
    double countsPerGearRev = rpm * gearRatio;
    double countsPerDegree = countsPerGearRev/360;

    double speed = 1;
    
    public void init(HardwareMap hwMap)
    {
        armLift = hwMap.get(DcMotor.class, "armLiftMotor");
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setDirection(DcMotor.Direction.FORWARD);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        extension = hwMap.get(DcMotor.class, "extension_1");
    }
    
    public void ResetEncoders()
    {
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    /*public void UpwardPosition()
    {
        armLift.setTargetPosition(-2200);
        if (armLift.getCurrentPosition() > -2200)
        {
            armLift.setPower(speed);
        }
        else
        {
            armLift.setPower(0);
        }
    }
    
    public void DownwardPosition()
    {
        armLift.setTargetPosition(300);
        if (armLift.getCurrentPosition() < 300)
        {
            armLift.setPower(-speed);
        }
        else
        {
            armLift.setPower(0);
        }
    }*/
    
    public void SetSpeed(double change)
    {
        speed = change;
    }
   
    public void rotate(double targetPower, String mode)
    {    
        int armPosition = 0;
        if (mode == "T")
        {
            if (targetPower > 0.1)
            {
                armLift.setTargetPosition(-2500);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setPower(speed);
            }
            else if (targetPower < -0.1)
            {
                armLift.setTargetPosition(300);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setPower(speed);
            }
            else
            {
                /*if (armLift.getCurrentPosition() < 100 || armLift.getCurrentPosition() > 700) // tune values?
                {
                    armLift.setTargetPosition(0);
                    armLift.setPower(0);
                }
                else
                {*/
                    armLift.setTargetPosition(armLift.getCurrentPosition() + ((int)countsPerDegree * 1));
                    armLift.setPower(speed);
                //}
            }
        }
        else if (mode == "A")
        {
        /*
        needed locations:
        0: 0
        5: -17
        30:-909
        45: -1212
        70: -1901
        90: -2497
        */
        //278
            //armPosition = (int)(countsPerDegree * targetPower);
            //armLift.setTargetPosition(armPosition);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            switch ((int)targetPower){
                case -15:
                    armLift.setTargetPosition(278);
                    if (armLift.getCurrentPosition() < 278 && extension.getCurrentPosition() > 150)
                    {
                        armLift.setPower(-speed);
                    }
                    else
                    {
                        armLift.setPower(speed);
                    }
                    break;
                case 5:
                    armLift.setTargetPosition(-17);
                    if (armLift.getCurrentPosition()>-17){
                        armLift.setPower(-speed);
                    }
                    else
                    {
                        armLift.setPower(speed);
                    }
                    break;
                case 30:
                    armLift.setTargetPosition(-909);
                    if (armLift.getCurrentPosition()>-909){
                        armLift.setPower(-speed);
                    }
                    else
                    {
                        armLift.setPower(speed);
                    }
                    break;
                case 45:
                    armLift.setTargetPosition(-1212);
                    if (armLift.getCurrentPosition()>-1212){
                        armLift.setPower(-speed);
                    }
                    else
                    {
                        armLift.setPower(speed);
                    }
                    break;
                case 60:
                    armLift.setTargetPosition(-1700);
                    if (armLift.getCurrentPosition()>-1700){
                        armLift.setPower(-speed);
                    }
                    else
                    {
                        armLift.setPower(speed);
                    }
                    break;
                case 70:
                    armLift.setTargetPosition(-1901);
                    if (armLift.getCurrentPosition()>-1901){
                        armLift.setPower(-speed);
                    }
                    else
                    {
                        armLift.setPower(speed);
                    }
                    break;
                case 80:
                    armLift.setTargetPosition(-2000);
                    if (armLift.getCurrentPosition()>-2000){
                        armLift.setPower(-speed);
                    }
                    else
                    {
                        armLift.setPower(speed);
                    }
                    break;
                case 90:
                    armLift.setTargetPosition(-2200);
                    if (armLift.getCurrentPosition()>-2200){
                        armLift.setPower(-speed);
                    }
                    else
                    {
                        armLift.setPower(speed);
                    }
                    break;
            }
            /*if (armLift.getCurrentPosition() < armPosition)
            {
                armLift.setPower(-speed);
            }
            else
            {
                armLift.setPower(speed);
            }*/
        }
    }
}

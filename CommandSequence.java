package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.MoveCommand;
import org.firstinspires.ftc.teamcode.OdometrySensor;
import org.firstinspires.ftc.teamcode.ClawServo;

@Autonomous
public class CommandSequence extends LinearOpMode
{
    MoveCommand moveCmd = new MoveCommand();
    Drivetrain dt = new Drivetrain();
    OdometrySensor odo = new OdometrySensor();
    ClawServo cs = new ClawServo();
    
    boolean moving = true;
    double speed = 0.4;

    double halfWidth = 7.625;
    double halfLength = 7.5;
    
    double grabDistance = 24.75; // old 22.8 
    int grabHeight = 210;
    
    // use later for scalability
    // double halfLength = 8.625; //y
    // double halfWidth = 7.625; //x
    
    private void ContinueMovement()
    {
        moving = true;
    }

    private void Stop()
    {
        dt.FieldOrientedTranslate(0,0,0,0);
        moving = false;
    }
    
    private void CommandSequence()
    {
        /* Parameter order --
        double speed : range 0 - 1
        double x position : range > 0
        double y position : range > 0
        double rotation : range 0 - 360
        int tgtExtension : range 0 - 800
        int tgtArmRotation : range -15, 5, 30, 45, 50, 60, 70, 80, 90
        boolean tgtClawOpen
        char wristPosition : range 'N', 'D', 'B'
        */
        
        // length = 15 inches
        // width = 15.25 inches
        // height = 15 inches
        // 260 degree ROM
        
        // hook
        Hook(0);
        
        // push
        // moveCmd.MoveToPosition(speed,32,19,0,0,120,false,'N'); //4
        // moveCmd.MoveToPosition(speed,32,50,0,0,grabHeight,false,'G'); //5
        // moveCmd.MoveToPosition(speed,43,50,0,0,grabHeight,false,'G'); //6
        // moveCmd.MoveToPosition(speed,43,27,0,0,grabHeight,false,'G');
        // moveCmd.MoveToPosition(speed,43,grabDistance,0,0,grabHeight,false,'G'); //7
        // moveCmd.MoveToPosition(speed,43,grabDistance,0,0,grabHeight,true,'G'); //7
        // while (moveCmd.GetClawPositionReading() != 0.3) {moveCmd.UpdateClawPosition();}
        // sleep(300);
        
        // // grab
        // moveCmd.MoveToPosition(speed,43,grabDistance,0,0,160,true,'G'); //8
        // moveCmd.MoveToPosition(speed,0,6,0,0,50,true,'G'); //8
        
        // // hook
        // Hook(-2);
        
        // push
        // moveCmd.MoveToPosition(speed,32,19,0,0,120,false,'N'); //11
        // moveCmd.MoveToPosition(speed,40,19,0,0,120,false,'N'); //11
        // moveCmd.MoveToPosition(speed,40,50,0,0,grabHeight,false,'G'); //11
        // moveCmd.MoveToPosition(speed,51,50,0,0,grabHeight,false,'G'); //12
        // moveCmd.MoveToPosition(speed,51,27,0,0,grabHeight,false,'G');
        // moveCmd.MoveToPosition(speed,51,grabDistance,0,0,grabHeight,false,'G'); //13
        // moveCmd.MoveToPosition(speed,51,grabDistance,0,0,grabHeight,true,'G'); //7
        // while (moveCmd.GetClawPositionReading() != 0.3) {moveCmd.UpdateClawPosition();}
        // sleep(300);
        
        // // grab
        // moveCmd.MoveToPosition(speed,51,grabDistance,0,0,160,true,'G');
        // moveCmd.MoveToPosition(speed,0,10,0,0,50,true,'G'); //8
    
        // // hook
        // Hook(-4);

        // // push
        // moveCmd.MoveToPosition(speed,50,19,0,0,120,false,'N'); //11
        // moveCmd.MoveToPosition(speed,50,50,0,0,grabHeight,false,'G'); //17
        // moveCmd.MoveToPosition(speed,56,50,0,0,grabHeight,false,'G'); //18
        // moveCmd.MoveToPosition(speed,56,27,0,0,grabHeight,false,'G');
        // moveCmd.MoveToPosition(speed,56,grabDistance,0,0,grabHeight,false,'G'); //19
        // moveCmd.MoveToPosition(speed,56,grabDistance,0,0,grabHeight,true,'G'); //19
        // while (moveCmd.GetClawPositionReading() != 0.3) {moveCmd.UpdateClawPosition();}
        // sleep(300);
        
        // // grab
        // moveCmd.MoveToPosition(speed,56,grabDistance,0,0,160,true,'G'); //20
        // moveCmd.MoveToPosition(speed,0,10,0,0,50,true,'G'); //8

        // // hook
        // Hook(6);

        // //grab
        // Grab();

        // // hook
        // Hook(8);g

        // end sequence
        moveCmd.MoveToPosition(speed,50,4,0,0,0,false,'G');
        Stop();
    }

    private void Hook(int offset) // add offset constant
    {
        // deprecated
        // moveCmd.MoveToPosition(speed,0,-12,0,0,150,true,'D'); //1
        // moveCmd.MoveToPosition(speed,0,-24,0,0,150,true,'D'); //1
        // moveCmd.MoveToPosition(speed,0,-24,0,0,150,true,'B'); //1
        // moveCmd.MoveToPosition(speed,0,-21.5,0,0,150,true,'B'); //1
        // deprecated
        // moveCmd.MoveToPosition(speed,0,-2,0,0,0,true,'D'); //1
        // moveCmd.MoveToPosition(speed,0,-6,0,0,130,true,'D');
        // moveCmd.MoveToPosition(speed,0,-18,0,0,130,true,'D'); //1
        // moveCmd.MoveToPosition(speed,0,-18,0,650,130,true,'D'); //1
        moveCmd.MoveToPosition(speed,offset,30,0,0,50,true,'B');
        moveCmd.MoveToPosition(speed,offset,30,0,0,67,true,'B');
        moveCmd.MoveToPosition(speed,offset,26.5,0,0,67,true,'B');
    }

    private void Grab()
    {
        moveCmd.MoveToPosition(speed,43,27,0,0,grabHeight,false,'G');
        moveCmd.MoveToPosition(speed,43,grabDistance,0,0,grabHeight,false,'G'); //23
        moveCmd.MoveToPosition(speed,43,grabDistance,0,0,grabHeight,true,'G');
        while (moveCmd.GetClawPositionReading() != 0.3) {moveCmd.UpdateClawPosition();}
        sleep(300);
        moveCmd.MoveToPosition(speed,43,grabDistance,0,0,160,true,'G'); //23
        moveCmd.MoveToPosition(speed,0,10,0,0,50,true,'G'); //8
    }
    
    private void BasicPark()
    {
        dt.Translate(0.5,0,0);
        cs.clawMove(true);
        cs.Update();
        sleep(7500);
        Stop();
    }

    private void TestSequence()
    {
        // moveCmd.MoveToPosition(speed,0,6,0,0,0,true,'N');
        moveCmd.MoveToPosition(speed,0,-12,0,0,0,false,'N');
        //odo.OdometryControl(speed,0,24,0);
        telemetry.addData("x", odo.GetPositionX());
        telemetry.addData("y", odo.GetPositionY());
        // telemetry.addData("h", odo.GetImuReading());
        // telemetry.addData("ex", odo.GettErrorX());
        // telemetry.addData("ey", odo.GetErrorY());
        // telemetry.addData("eh", odo.GetErrorH());
        // telemetry.addData("ox", odo.GetOutputX());
        // telemetry.addData("oy", odo.GetOutputY());
        // telemetry.addData("oh", odo.GetOutputH());
        telemetry.update();
    }
    
    @Override
    public void runOpMode() 
    {
        dt.init(hardwareMap);
        moveCmd.init(hardwareMap, true);
        odo.init(hardwareMap,true);
        cs.init(hardwareMap);
        
        double volts = hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        double normalizedVolts = (1 - volts/14) + 0.57; // reduced to account for volt drops during auton
        speed *= normalizedVolts;
        
        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                // maybe add arm speed change when hover/moving
                
                //CommandSequence();
                TestSequence();
                //BasicPark();
            }
        }
    }
}

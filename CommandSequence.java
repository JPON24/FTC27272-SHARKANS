package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.MoveCommand;
import org.firstinspires.ftc.teamcode.OdometrySensor;

@Autonomous
public class CommandSequence extends LinearOpMode
{
    MoveCommand moveCmd = new MoveCommand();
    Drivetrain dt = new Drivetrain();
    OdometrySensor odo = new OdometrySensor();
    
    boolean moving = true;
    double speed = 1;

    double halfWidth = 7.625;
    double halfLength = 7.5;
    
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
        Hook();
        
        // push
        moveCmd.MoveToPosition(speed,32,19,0,0,120,false,'N'); //4
        moveCmd.MoveToPosition(speed,32,52,0,0,200,false,'G'); //5
        moveCmd.MoveToPosition(speed,43,52,0,0,200,false,'G'); //6
        moveCmd.MoveToPosition(speed,43,24,0,0,190,false,'G');
        moveCmd.MoveToPosition(0.2,42,21,0,0,190,false,'G'); //7
        moveCmd.MoveToPosition(0.2,42,21,0,0,190,true,'G'); //7
        while (moveCmd.GetClawPositionReading() != 0.3) {moveCmd.UpdateClawPosition();}

        // grab
        moveCmd.MoveToPosition(speed,42,21,0,0,160,true,'G'); //8
        moveCmd.MoveToPosition(speed,0,10,0,0,50,true,'G'); //8
        
        // hook
        Hook();
        moveCmd.MoveToPosition(speed,39,4,0,0,0,false,'G');
        Stop();
        
        // push
        // moveCmd.MoveToPosition(speed,32,19,0,0,120,false,'N'); //11
        // moveCmd.MoveToPosition(speed,40,19,0,0,120,false,'N'); //11
        // moveCmd.MoveToPosition(speed,40,52,0,0,200,false,'G'); //11
        // moveCmd.MoveToPosition(speed,51,52,0,0,200,false,'G'); //12
        // moveCmd.MoveToPosition(speed,51,24,0,0,190,false,'G');
        // moveCmd.MoveToPosition(0.2,51,21,0,0,190,false,'G'); //13
        // moveCmd.MoveToPosition(0.2,51,21,0,0,190,true,'G'); //7
        // while (moveCmd.GetClawPositionReading() != 0.3) {moveCmd.UpdateClawPosition();}
        
        // // grab
        // moveCmd.MoveToPosition(speed,51,21,0,0,160,true,'G');
        // moveCmd.MoveToPosition(speed,0,10,0,0,50,true,'G'); //8
        
    
        // // hook
        // Hook();

        // // push
        // moveCmd.MoveToPosition(speed,50,19,0,0,120,false,'N'); //11
        // moveCmd.MoveToPosition(speed,50,52,0,0,200,false,'G'); //17
        // moveCmd.MoveToPosition(speed,56,52,0,0,200,false,'G'); //18
        // moveCmd.MoveToPosition(speed,56,24,0,0,190,false,'G');
        // moveCmd.MoveToPosition(0.2,56,21,0,0,190,false,'G'); //19
        // moveCmd.MoveToPosition(0.2,56,21,0,0,190,true,'G'); //19
        // while (moveCmd.GetClawPositionReading() != 0.3) {moveCmd.UpdateClawPosition();}

        // // grab
        // moveCmd.MoveToPosition(speed,56,21,0,0,160,true,'G'); //20
        // moveCmd.MoveToPosition(speed,0,10,0,0,50,true,'G'); //8

        // // hook
        // Hook();

        // //grab
        // Grab();

        // // hook
        // Hook();

        // // end sequence
        // moveCmd.MoveToPosition(speed,39,4,0,0,0,false,'G');
        // Stop();
    }

    private void Hook() // add offset constant
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
        moveCmd.MoveToPosition(speed,0,34.5,0,0,50,true,'B');
        moveCmd.MoveToPosition(speed,0,34.5,0,0,69,true,'B');
        moveCmd.MoveToPosition(speed,0,31.5,0,0,69,true,'B');
    }

    private void Grab()
    {
        moveCmd.MoveToPosition(speed,31,24,0,0,190,false,'G');
        moveCmd.MoveToPosition(0.2,31,21,0,0,190,false,'G'); //23
        moveCmd.MoveToPosition(0.2,31,21,0,0,190,true,'G');
        while (moveCmd.GetClawPositionReading() != 0.3) {moveCmd.UpdateClawPosition();}
        moveCmd.MoveToPosition(speed,31,21,0,0,160,true,'G'); //23
        moveCmd.MoveToPosition(speed,0,10,0,0,50,true,'G'); //8
    }

    private void TestSequence()
    {
        // moveCmd.MoveToPosition(speed,0,6,0,0,0,true,'N');
        // moveCmd.MoveToPosition(speed,6,0,0,0,0,false,'N');
        odo.OdometryControl(speed,0,24,0);
        telemetry.addData("x", odo.GetPositionX());
        telemetry.addData("y", odo.GetPositionY());
        telemetry.addData("h", odo.GetImuReading());
        telemetry.addData("ex", odo.GetErrorX());
        telemetry.addData("ey", odo.GetErrorY());
        telemetry.addData("eh", odo.GetErrorH());
        telemetry.addData("ox", odo.GetOutputX());
        telemetry.addData("oy", odo.GetOutputY());
        telemetry.addData("oh", odo.GetOutputH());
        telemetry.update();
    }
    
    @Override
    public void runOpMode() 
    {
        dt.init(hardwareMap);
        moveCmd.init(hardwareMap, true);
        odo.init(hardwareMap,true);
        
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
                
                // CommandSequence();
                TestSequence();
            }
        }
    }
}

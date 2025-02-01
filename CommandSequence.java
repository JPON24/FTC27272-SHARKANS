package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.MoveCommand;

@Autonomous
public class CommandSequence extends LinearOpMode
{
    MoveCommand moveCmd = new MoveCommand();
    Drivetrain dt = new Drivetrain();
    
    boolean moving = true;
    double speed = 0.5;

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
        moveCmd.MoveToPosition(speed,-26,-19,0,0,110,false,'N'); //4
        moveCmd.MoveToPosition(speed,-26,-48,0,0,60,false,'N'); //5
        moveCmd.MoveToPosition(speed,-37,-48,0,0,50,false,'N'); //6
        moveCmd.MoveToPosition(speed,-37,-8,0,0,30,false,'N'); //7

        // grab
        moveCmd.MoveToPosition(speed,-37,-8,0,0,30,true,'N'); //8
        
        // hook
        Hook();
        
        // push
        moveCmd.MoveToPosition(speed,-36,-19,0,0,110,false,'N'); //11
        moveCmd.MoveToPosition(speed,-36,-48,0,0,60,true,'N'); //11
        moveCmd.MoveToPosition(speed,-47,-48,0,0,50,true,'N'); //12
        moveCmd.MoveToPosition(speed,-47,-9,0,0,30,false,'N'); //13
        
        // grab
        moveCmd.MoveToPosition(speed,-47,-9,0,0,60,true,'N');
    
        // hook
        Hook();

        // push
        moveCmd.MoveToPosition(speed,-46,-19,0,0,60,false,'N'); //11
        moveCmd.MoveToPosition(speed,-46,-48,0,0,40,true,'N'); //17
        moveCmd.MoveToPosition(speed,-52,-48,0,0,30,true,'N'); //18
        moveCmd.MoveToPosition(speed,-52,-8,0,0,30,true,'N'); //19

        // grab
        moveCmd.MoveToPosition(speed,-52,-8,0,0,30,false,'N'); //20
        moveCmd.MoveToPosition(speed,-52,-8,0,0,30,true,'N'); //20

        // hook
        Hook();

        //grab
        Grab();

        // hook
        Hook();

        // end sequence
        moveCmd.MoveToPosition(speed,-35,-4,0,0,0,false,'N');
        Stop();
    }

    private void Hook() // add offset constant
    {
        // deprecated
        // moveCmd.MoveToPosition(speed,0,-12,0,0,150,true,'D'); //1
        // moveCmd.MoveToPosition(speed,0,-24,0,0,150,true,'D'); //1
        // moveCmd.MoveToPosition(speed,0,-24,0,0,150,true,'B'); //1
        // moveCmd.MoveToPosition(speed,0,-21.5,0,0,150,true,'B'); //1
        moveCmd.MoveToPosition(speed,0,-2,0,0,0,true,'D'); //1
        moveCmd.MoveToPosition(speed,0,-6,0,0,130,true,'D');
        moveCmd.MoveToPosition(speed,0,-18,0,0,130,true,'D'); //1
        moveCmd.MoveToPosition(speed,0,-18,0,650,130,true,'D'); //1
    }

    private void Grab()
    {
        moveCmd.MoveToPosition(speed,-27.5,-8,0,0,30,false,'N'); //23
        moveCmd.MoveToPosition(speed,-27.5,-8,0,0,30,true,'N'); //23
    }

    private void TestSequence()
    {
        moveCmd.MoveToPosition(speed,0,0,0,0,0,true,'N');
        // moveCmd.MoveToPosition(speed,0,-16,0,0,40,false,'N');
    }
    
    @Override
    public void runOpMode() 
    {
        dt.init(hardwareMap);
        moveCmd.init(hardwareMap, true);
        
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
                
                CommandSequence();
                // TestSequence();
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.MoveCommand;
import org.firstinspires.ftc.teamcode.OdometrySensor;
import org.firstinspires.ftc.teamcode.ClawServo;

@Config
@Autonomous
public class CommandSequence extends LinearOpMode
{
    MoveCommand moveCmd = new MoveCommand();
    Drivetrain dt = new Drivetrain();
    OdometrySensor odo = new OdometrySensor();
    ClawServo cs = new ClawServo();
    
    boolean moving = true;
    double speed = 1; // 1.1

    double halfWidth = 7.625;
    double halfLength = 7.625;
    
    double grabDistance = 3.5; // old 4.5
    int grabHeight = -700;
    
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
        char wristPosition : range 'N', 'D', 'B','G'
        */
        
        // length = 15 inches
        // width = 15.25 inches
        // height = 15 inches
        // 260 degree ROM
        
        
        // hook  
        Hook(-7);
        
        // push
        Grab(0);
        moveCmd.MoveToPosition(1,-2,6,0,0,-4900,true,'G');
        
        // hook
        Hook(-2);
        
        // push
        Grab(10);  
        moveCmd.MoveToPosition(1,3,6,0,0,-4900,true,'G');
    
        // hook
        Hook(3);

        Grab(0);

        moveCmd.MoveToPosition(1,0,6,0,0,-4900,true,'G');
        Hook(0);
        
        moveCmd.MoveToPosition(1,41,18,0,0,0,false,'D');
        moveCmd.MoveToPosition(1,41,6,0,0,0,false,'D');
        // moveCmd.MoveToPosition(1,41,18,0,0,grabHeight,false,'G');
        // moveCmd.MoveToPosition(0.8,41,grabDistance,0,0,grabHeight,false,'G'); //7
        // moveCmd.MoveToPosition(0.8,41,grabDistance,0,0,grabHeight,true,'G'); //7
        // sleep(150);
        // moveCmd.MoveToPosition(0.8,41,grabDistance,0,0,0,true,'G'); //8
        
        // moveCmd.MoveToPosition(1,8,12,0,0,-4900,true,'G');
        // // // push
        // // Grab(15);

        // // hook
        // Hook(8);

        // // end sequence
        // moveCmd.MoveToPosition(1,0,8,0,0,-700,false,'G');
        // while (true)
        // {
        //     moveCmd.MoveToPosition(1,0,8,0,360,-700,false,'G');
        //     moveCmd.MoveToPosition(1,0,8,0,0,-700,false,'G');
        // }
        // moveCmd.MoveToPosition(1,50,8,0,0,-700,false,'G');
        Stop();
    }

    private void Hook(int offset) // add offset constant
    {
        moveCmd.MoveToPosition(1,offset,15,0,0,-4575,true,'G'); //1
        moveCmd.MoveToPosition(1,offset,26,0,0,-4575,true,'B'); //1
        moveCmd.MoveToPosition(1,offset,17,0,0,-4575,false,'B'); //1

//        moveCmd.MoveToPosition(1,offset,15,0,0,-4900,true,'G'); //1
//        moveCmd.MoveToPosition(0.75,offset,24,0,0,-4900,true,'G'); //1
//        moveCmd.MoveToPosition(0.75,offset,24,0,0,-4575,true,'B'); //1
//        moveCmd.MoveToPosition(0.75,offset,17,0,0,-4575,true,'B'); //1
//        moveCmd.MoveToPosition(1,offset,10,0,0,-4575,false,'B');
    }
    

    private void Grab(double offset)
    {
        moveCmd.MoveToPosition(1,32 + offset,17,0,0,-2000,false,'B'); //4
        moveCmd.MoveToPosition(1,32 + offset,48,0,0,grabHeight,false,'G'); //5
        moveCmd.MoveToPosition(1,41 + offset,48,0,0,grabHeight,false,'G'); //6
        moveCmd.MoveToPosition(1,41 + offset,28,0,0,grabHeight,false,'G');
        // moveCmd.MoveToPosition(1,41,18,0,0,grabHeight,false,'G');
        moveCmd.MoveToPosition(1,41 + offset,grabDistance,0,0,grabHeight,false,'G'); //7
        moveCmd.MoveToPosition(1,41 + offset,grabDistance,0,0,grabHeight,true,'G'); //7
        sleep(150);
        moveCmd.MoveToPosition(1,41 + offset,grabDistance,0,0,-1500,true,'G'); //8
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
        // moveCmd.MoveToPosition(speed,0,12,0,0,0,true,'N');
        // moveCmd.MoveToPosition(speed,-24,0,0,0,0,false,'N');
        
        // moveCmd.MoveToPosition(speed,0,0,0,0,0,true,'G');
        // moveCmd.MoveToPosition(speed,0,12,0,0,0,true,'G');
        // moveCmd.MoveToPosition(speed,12,12,0,0,0,true,'G');
        // moveCmd.MoveToPosition(speed,12,0,0,0,0,true,'G');

        odo.OdometryControl(speed,0,0,180);
        telemetry.addData("x", odo.GetPositionX());
        telemetry.addData("y", odo.GetPositionY());
        telemetry.addData("h", odo.GetImuReading());
        telemetry.addData("ex", odo.GetErrorX());
        telemetry.addData("ey", odo.GetErrorY());
        telemetry.addData("eh", odo.GetErrorH());
        telemetry.addData("ox", odo.GetOutputX());
        telemetry.addData("oy", odo.GetOutputY());
        telemetry.addData("oh", odo.GetOutputH());
        telemetry.addData("PROPORTIONAL", odo.GetPorportionalY());
        telemetry.addData("INTEGRAL", odo.GetIntegralY());
        telemetry.addData("DERIVATIVE", odo.GetDerivativeY());
        telemetry.update();
    }
    
    @Override
    public void runOpMode() 
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        dt.init(hardwareMap);
        moveCmd.init(hardwareMap, true);
        odo.init(hardwareMap,true);
//        cs.init(hardwareMap, true);

        double volts = hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        double normalizedVolts = (1 - volts/14) + volts/14 * 0.857; // reduced to account for volt drops during auton
        speed *= normalizedVolts;
        
        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                // maybe add arm speed change when hover/moving
                
//                 CommandSequence();
                TestSequence();
                // BasicPark();
            }
        }
    }
}

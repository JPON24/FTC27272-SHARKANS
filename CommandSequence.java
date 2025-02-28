package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    double grabDistance = 7; // old 4.5
    int grabHeight = -600;
    int hookHeight = -4730;

    double initialMoveInDistance = 16;
    
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

    private void SpecimenSequence()
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

        moveCmd.MoveToPosition(speed,0,18,0,0,hookHeight,true,'B'); //1
        Hook(0);
        moveCmd.MoveToPosition(speed,32,20,0,0,-2850,false,'G'); //5
        Push(0);
        Push(10);
        moveCmd.MoveToPosition(speed,52,48,0,0,grabHeight,false,'G'); //5
        moveCmd.MoveToPosition(speed,58,48,0,0,grabHeight,false,'G'); //6

        Grab(16);
        moveCmd.MoveToPosition(speed,-2,initialMoveInDistance,0,0,hookHeight,true,'B');

        Hook(-2);

        Grab(0);
        moveCmd.MoveToPosition(speed,2,initialMoveInDistance,0,0,hookHeight,true,'B');

        Hook(2);

        Grab(0);
        moveCmd.MoveToPosition(speed,4,initialMoveInDistance,0,0,hookHeight,true,'B');
        Hook(4);

        Grab(0);
        moveCmd.MoveToPosition(speed,6,initialMoveInDistance,0,0,hookHeight,true,'B');
        Hook(6);

        moveCmd.MoveToPosition(speed,41,6,0,0,0,false,'/');
        Stop();
    }

    private void Hook(int offset) // add offset constant
    {
        moveCmd.MoveToPosition(1, 0 + offset, 16, 0, 0,hookHeight, true, 'B');
        moveCmd.MoveToPosition(1, 0 + offset, 26, 0, 0,hookHeight, true, 'B');
        moveCmd.MoveToPosition(1, 0 + offset, 20, 0, 0,hookHeight, false, 'B');
    }

    private void Grab(double offset)
    {
        moveCmd.MoveToPosition(1, 42, grabDistance, 0, 0,grabHeight, false, 'G'); //7
        moveCmd.MoveToPosition(1, 42, grabDistance, 0, 0,grabHeight, true, 'G'); //7
        moveCmd.MoveToPosition(1, 42, grabDistance, 0, 0,-900, true, 'G'); //7
//        moveCmd.MoveToPosition(speed,42 + offset,grabDistance,0,0,-850,true,'G'); //8
    }

    private void Push(double offset)
    {
        if (offset != 0) {moveCmd.MoveToPosition(speed,32 + offset,48,0,0,grabHeight,false,'G'); }
        else {moveCmd.MoveToPosition(speed,32 + offset,48,0,0,-1500,false,'G'); }
        moveCmd.MoveToPosition(speed,42 + offset,48,0,0,grabHeight,false,'G'); //6
        moveCmd.MoveToPosition(speed,42 + offset,8,0,0,grabHeight,false,'G'); //6
    }

    private void BasicPark()
    {
        dt.Translate(0.5,0,0);
        cs.SetClawOpen(true);
        cs.Update();
        sleep(7500);
        Stop();
    }

    private void TestSequence()
    {
        odo.OdometryControl(speed, 0, 0, 180);
        UpdateTelemetry();
    }

    private void BasketSequence()
    {
        DunkBasket();
        GrabSpec(0);
        DunkBasket();
        GrabSpec(10);
        DunkBasket();
        GrabSpec(21);
        FirstLevelAscent();
    }

    private void DunkBasket()
    {
        moveCmd.MoveToPosition(speed,-12,12,135,0,-3000,true,'D');
        moveCmd.MoveToPosition(speed,-12,12,135,0,-3000,false,'D');
    }

    private void GrabSpec(double offset)
    {
        moveCmd.MoveToPosition(speed, -7 - offset, 26,0,0,-6500,false,'M');
        moveCmd.MoveToPosition(speed, -7 - offset, 26,0,0,-6500,true,'M');
    }

    private void FirstLevelAscent()
    {
        moveCmd.MoveToPosition(speed, 0, 52, 0, 0, -3000, false, '/');
        moveCmd.MoveToPosition(speed, 0, 52, 90, 0, -3000, false, '/');
        moveCmd.MoveToPosition(speed, 12, 52, 0, 0, -4500, false, '/');
    }

    private void UpdateTelemetry()
    {
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
        cs.init(hardwareMap, true);

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
                SpecimenSequence();
//                BasketSequence();
//                TestSequence();
                // BasicPark();
            }
        }
    }
}

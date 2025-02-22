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
    
    double grabDistance = 4; // old 4.5
    int grabHeight = -700;
    int hookHeight = -4475;

    double initialMoveInDistance = 14;
    
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
    
    private void Sequence()
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

        moveCmd.MoveToPosition(speed,0,15,0,0,hookHeight,true,'G'); //1
        Hook(0);
        moveCmd.MoveToPosition(speed,32,20,0,0,-2500,false,'G'); //5

        Push(0);
        Push(10);
        moveCmd.MoveToPosition(speed,52,48,0,0,grabHeight,false,'G'); //5
        moveCmd.MoveToPosition(speed,57,48,0,0,grabHeight,false,'G'); //6

        Grab(15);
        moveCmd.MoveToPosition(speed,4,initialMoveInDistance,0,0,hookHeight,true,'G');

        Hook(4);

        Grab(0);
        moveCmd.MoveToPosition(speed,-4,initialMoveInDistance,0,0,hookHeight,true,'G');

        Hook(-3);

        Grab(0);
        moveCmd.MoveToPosition(speed,-8,initialMoveInDistance,0,0,hookHeight,true,'G');
        Hook(-6);

        Grab(0);
        moveCmd.MoveToPosition(speed,-12,initialMoveInDistance,0,0,hookHeight,true,'G');
        Hook(-9);

        moveCmd.MoveToPosition(speed,41,6,0,0,0,false,'D');
        Stop();
    }

    private void Hook(int offset) // add offset constant
    {
        moveCmd.MoveToPosition(speed,offset,26,0,0,hookHeight,true,'B'); //1
        moveCmd.MoveToPosition(speed,offset,23,0,0,hookHeight,false,'B'); //1S
    }

    private void Grab(double offset)
    {
        moveCmd.MoveToPosition(speed,42 + offset,grabDistance,0,0,grabHeight,false,'G'); //7
        moveCmd.MoveToPosition(0.5,42 + offset,grabDistance,0,0,grabHeight,true,'G'); //7
        moveCmd.MoveToPosition(speed,42 + offset,grabDistance,0,0,-1000,true,'G'); //8
    }

    private void Push(double offset)
    {
        moveCmd.MoveToPosition(speed,32 + offset,48,0,0,grabHeight,false,'G'); //5
        moveCmd.MoveToPosition(speed,42 + offset,48,0,0,grabHeight,false,'G'); //6
        moveCmd.MoveToPosition(speed,42 + offset,8,0,0,grabHeight,false,'G'); //6
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
        odo.OdometryControl(speed, 0, 0, 180);
        UpdateTelemetry();
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

                Sequence();
//                TestSequence();
                // BasicPark();
            }
        }
    }
}

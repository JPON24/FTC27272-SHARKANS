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

    double grabDistance = 4; // old 4.5
    int grabHeight = 190;
    int hookHeight = 1275;

    double runtimeXSumScalar = 0.01966; // 0.0115
    double runtimeYSumScalar = 0.0029; // 0.0115

    double preciseLenience = 0.5;
    double arcLenience = 2;



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
        Hook(0);
        moveCmd.MoveToPosition(speed,32,24,0,arcLenience,2,0.5,grabHeight,false,'G'); //5
        Push(0);
        Push(10);
        moveCmd.MoveToPosition(speed,52,44,0,arcLenience,2,0.5,grabHeight,false,'G'); //5
        moveCmd.MoveToPosition(speed,58,44,0,preciseLenience,2,0.5,grabHeight,false,'G'); //6

        Grab(18);

        Hook(6);

        Grab(0);

        Hook(4);

        Grab(0);
        Hook(2);

        Grab(0);
        Hook(-2);

        moveCmd.MoveToPosition(speed,41,6,0,arcLenience,2,0.5,0,false,'/');
        Stop();
    }

    private void Hook(int offset) // add offset constant
    {
        moveCmd.MoveToPosition(1, 0 + offset, 16, 0,arcLenience,2,.5,hookHeight, true, 'B');
        moveCmd.MoveToPosition(1, 0 + offset, 26.5, 0, preciseLenience,2,0.5,hookHeight, true, 'B');
        odo.OdometryControl(0,0 + offset, 26.5, 0, arcLenience,2);
        cs.SetClawOpen(false);
        cs.Update();
        sleep(150);
        odo.OdometryControl(0,0 + offset, 26.5, 0, arcLenience,2);
        cs.Update();
        cs.SetWristMode('S');
        sleep(150);
        moveCmd.MoveToPosition(1, 0 + offset, 16, 0,arcLenience,2,0.5, hookHeight, false, 'S');
    }

    private void Grab(double offset)
    {
        moveCmd.MoveToPosition(1, 40 + offset, 6, 0,0.5,2,arcLenience,grabHeight, false, 'G'); //7
        moveCmd.MoveToPosition(1, 40 + offset, grabDistance, 0,0.5,2,preciseLenience,grabHeight, false, 'G'); //7
        moveCmd.MoveToPosition(1, 40 + offset, grabDistance, 0,0.5,2,preciseLenience,grabHeight, true, 'G'); //7
        odo.OdometryControl(0,40 + offset, grabDistance, 0, 2,2);
        cs.SetClawOpen(true);
        sleep(150);
        moveCmd.MoveToPosition(1, 40 + offset, grabDistance, 0,1,2,preciseLenience,270, true, 'G'); //7
//        moveCmd.MoveToPosition(speed,42 + offset,grabDistance,0,0,-850,true,'G'); //8
    }

    // could try rotational push for speed
    private void Push(double offset)
    {
        moveCmd.MoveToPosition(speed,32 + offset,44,0,1,2,preciseLenience,grabHeight,false,'G');
        moveCmd.MoveToPosition(speed,42 + offset,44,0,1,2,preciseLenience,grabHeight,false,'G'); //6
        moveCmd.MoveToPosition(speed,42 + offset,10,0,1,2,arcLenience,grabHeight,false,'G'); //6
    }

    private void BasicPark()
    {
        dt.Translate(0.5,0,0);
        cs.SetClawOpen(true);
        cs.Update();
        sleep(7500);
        Stop();
    }

//    private void BasketSequence()
//    {
//        DunkBasket();
//        GrabSpec(0);
//        DunkBasket();
//        GrabSpec(10);
//        DunkBasket();
//        GrabSpec(21);
//        FirstLevelAscent();
//    }

//    private void DunkBasket()
//    {
//        moveCmd.MoveToPosition(speed,-12,12,135,0.5,750,true,'D');
//        moveCmd.MoveToPosition(speed,-12,12,135,0.5,750,false,'D');
//    }
//
//    private void GrabSpec(double offset)
//    {
//        moveCmd.MoveToPosition(speed, -7 - offset, 26,0,0.5,1650,false,'M');
//        moveCmd.MoveToPosition(speed, -7 - offset, 26,0,0.5,1650,true,'M');
//    }
//
//    private void FirstLevelAscent()
//    {
//        moveCmd.MoveToPosition(speed, 0, 52, 0, 0.5, 750, false, '/');
//        moveCmd.MoveToPosition(speed, 0, 52, 90, 0.5, 750, false, '/');
//        moveCmd.MoveToPosition(speed, 12, 52, 0, 0.5, 1125, false, '/');
//    }

    private void UpdateTelemetry()
    {
        telemetry.addData("x", odo.GetPositionX());
        telemetry.addData("y", odo.GetPositionY());
        telemetry.addData("h", odo.GetImuReading());
        telemetry.addData("ex", odo.GetErrorX());
        telemetry.addData("eh", odo.GetErrorH());
        telemetry.addData("ey", odo.GetErrorY());
        telemetry.addData("PROPORTIONAL", odo.GetPorportionalX());
        telemetry.addData("DERIVATIVE", odo.GetDerivativeX());
        telemetry.addData("INTEGRAL X", odo.GetIntegralSumX());
        telemetry.addData("INTEGRAL Y", odo.GetIntegralSumY());
        telemetry.update();
    }

    /*
    x = 2.36/120
    y = 1.45/500
     */

    private void TestSequenceUp()
    {
//        moveCmd.MoveToPosition(speed,-24,0,-90,0,false,'/');
//        UpdateTelemetry();
//        moveCmd.MoveToPosition(speed,-12,0,-90,0,false,'/');
        odo.TuningUp();
        moveCmd.MoveToPosition(speed,12,0,0,0.5,2,0.5,1275,false,'/');
        moveCmd.MoveToPosition(speed,12,12,0,0.5,2,0.5,1275,false,'/');
        moveCmd.MoveToPosition(speed,0,0,0,0.5,2,0.5,1275,false,'/');
//        odo.OdometryControl(speed, 12, 6, 0, 0.5);
        UpdateTelemetry();
    }

    private void TestSequenceDown()
    {
//        moveCmd.MoveToPosition(speed,-24,0,-90,0,false,'/');
//        UpdateTelemetry();
//        moveCmd.MoveToPosition(speed,-12,0,-90,0,false,'/');
        odo.TuningDown();
        moveCmd.MoveToPosition(speed,12,0,0,0.5,2,0.5,190,false,'/');
//        moveCmd.MoveToPosition(speed,12,12,0,0.5,2,0.5,190,false,'/');
//        moveCmd.MoveToPosition(speed,0,0,0,0.5,2,0.5,190,false,'/');
//        odo.OdometryControl(speed, 12, 6, 0, 0.5);
        UpdateTelemetry();
    }

    private void RotationSequence()
    {
        moveCmd.MoveToPosition(0,0,0,0,10,2,1,1275,false,'/');
        while (true)
        {
            moveCmd.MoveToPosition(speed,0,0,90,1,2,0.5,1275,false,'/');
        }
    }

    private void IntegralTest()
    {
        for (int i = 0; i < 5; i++)
        {
            moveCmd.MoveToPosition(speed,0,50,0,0.5,2,0,0,false,'/');
            moveCmd.MoveToPosition(speed,0,0,0,0.5,2,0,0,false,'/');
        }
        for (int i = 0; i < 5; i++)
        {
            moveCmd.MoveToPosition(speed,12,0,0,0.5,2,0,0,false,'/');
            moveCmd.MoveToPosition(speed,0,0,0,0.5,2,0,0,false,'/');
        }
        while (true)
        {
            dt.FieldOrientedTranslate(0,0,0,0);
            UpdateTelemetry();
        }
    }

    @Override
    public void runOpMode()
    {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        dt.init(hardwareMap);
        moveCmd.init(hardwareMap, true);
        odo.init(hardwareMap,true);
        cs.init(hardwareMap, true);

        double volts = hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        // removed for maximum speed possible
        double normalizedVolts = (1 - volts/14) + volts/14 * 0.857; // reduced to account for volt drops during auton
        speed *= normalizedVolts;

        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                // maybe add arm speed change when hover/moving
//                SpecimenSequence();
//                BasketSequence();
//                TestSequenceUp();
                TestSequenceDown();
//                IntegralTest();
//                RotationSequence();
                // BasicPark();
            }
        }
    }
}

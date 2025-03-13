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
    ArmLiftMotor am = new ArmLiftMotor();

    boolean moving = true;
    double speed = 1; // 1.1

    double grabDistance = 7; // old 4.5
    double hookDistance = 24; // 22.5
    int grabHeight = 145;
    int hookHeight = 1250;

    double preciseLenience = 0.6;
    double arcLenience = 10;

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
        moveCmd.MoveToPosition(speed,32,19,0,arcLenience,2,0.5,grabHeight,false,'G'); //5
        Push(0);
        Push(10);
        moveCmd.MoveToPosition(speed,52,47,0,arcLenience,1,0.5,grabHeight,false,'G'); //5
        moveCmd.MoveToPosition(speed,58,47,0,preciseLenience,2,0.5,grabHeight,false,'G'); //6
        moveCmd.MoveToPosition(speed,58,12,0,preciseLenience,1,0.5,grabHeight,false,'G'); //6

        Grab(0);
        Hook(4);

        Grab(0);
        Hook(4);

        Grab(0);
        Hook(4);

        Grab(0);
        Hook(4);

//        Grab(0);
//        Hook(4);

        moveCmd.MoveToPosition(speed,40,10,0,0.25 ,2,0.5,0,false,'G');
        Stop();
    }

    private void Hook(int offset) // add offset constant
    {
        moveCmd.MoveToPosition(1, 0 + offset, grabDistance + 2, 0,preciseLenience,0,1,1250, true, 'B');
        moveCmd.MoveToPosition(0.8, 0 + offset, hookDistance, 0,preciseLenience,1,1, 1250, true, 'B');
//        moveCmd.MoveToPosition(0.8, 0 + offset - 4, hookDistance, 0,preciseLenience,0,0.5, 1250, true, 'B');
//            TeleopMoveCommandY(0.5, 0 + localOffset, 23.5, 0,preciseLenience,1,0.5, 1275, true, 'B');
//        odo.OdometryControl(1, offset-3, hookDistance,0,preciseLenience,0);
        dt.FieldOrientedTranslate(-1,0,0,odo.GetImuReading());
//            s1.OdometryControl(0,0 + localOffset, 23, 0, arcLenience);
        cs.SetClawOpen(false);
        cs.Update();
        sleep(150);
        dt.FieldOrientedTranslate(0,0,0,0);
        cs.Update();
        cs.SetWristMode('S');
        sleep(150);
        moveCmd.MoveToPosition(1, 0 + offset, 14, 0, preciseLenience,1,0.5,1250, false, 'S');
    }

    private void Grab(double offset)
    {
        moveCmd.MoveToPosition(1, 40 + offset, grabDistance + 4, 0,1,2,0.5, 145, false, 'G');
        moveCmd.MoveToPosition(0.6, 40 + offset, grabDistance, 0,preciseLenience, 2,0.5,145, false, 'G');
        moveCmd.MoveToPosition(0.5, 40 + offset, grabDistance, 0,arcLenience, 2,0.5,145, true, 'G');
//            s1.OdometryControl(0,40, grabDistance, 0,arcLenience);
        dt.FieldOrientedTranslate(0,0,0,0);
        cs.SetClawOpen(true);
        sleep(200);
        moveCmd.MoveToPosition(0.5, 40 + offset, grabDistance,0,preciseLenience,1,1,1250, true, 'G');
        moveCmd.MoveToPosition(1, 40 + offset, grabDistance + 2, 0,preciseLenience,2,1,1250, true, 'B');
//        moveCmd.MoveToPosition(speed,42 + offset,grabDistance,0,0,-850,true,'G'); //8
    }

    // could try rotational push for speed
    private void Push(double offset)
    {
        moveCmd.MoveToPosition(speed,32 + offset,47,0,1,1,preciseLenience,grabHeight,false,'G');
        moveCmd.MoveToPosition(speed,42 + offset,47,0,1,2,preciseLenience,grabHeight,false,'G'); //6
        moveCmd.MoveToPosition(speed,42 + offset,12,0,1,1,preciseLenience,grabHeight,false,'G'); //6
    }

    private void BasicPark()
    {
        dt.Translate(0.5,0,0);
        cs.SetClawOpen(true);
        cs.Update();
        sleep(7500);
        Stop();
    }

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
//        telemetry.addData("INTEGRAL X", odo.GetIntegralSumX());
//        telemetry.addData("INTEGRAL Y", odo.GetIntegralSumY());
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
//        moveCmd.MoveToPosition(speed,12,0,0,0.5,2,0.5,190,false,'/');

        odo.OdometryControl(1,12,0,0,0.5,2);

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
        am.init(hardwareMap);

        cs.SetDiffPos(0.44, 0.99);
        cs.SetClawOpen(true);
        cs.Update();
//        am.Rotate(0,'T');
//        am.SetLocalNeutral(50);
//        am.Rotate(0,'T');

        double volts = hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
        // removed for maximum speed possible
//        double normalizedVolts = (1 - volts/14) + volts/14 * 0.857; // reduced to account for volt drops during auton
//        speed *= normalizedVolts;
        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                // maybe add arm speed change when hover/moving
                SpecimenSequence();
//                BasketSequence();
//                TestSequenceUp();
//                TestSequenceDown();
//                IntegralTest();
//                RotationSequence();
                // BasicPark();
            }
        }
    }
}

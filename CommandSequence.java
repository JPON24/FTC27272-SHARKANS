package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Config
@Autonomous
public class CommandSequence extends LinearOpMode
{
    MoveCommand moveCmd = new MoveCommand();
    Drivetrain dt = new Drivetrain();
    SharkDrive shark = new SharkDrive();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();

    boolean moving = true;
    double speed = 1; // 1.1

    double grabDistance = 8; // old 4.5
    double hookDistance = 25; // 22.5
    int grabHeight = 120;
    int hookHeight = 1275;

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
        moveCmd.MoveToPosition(speed,32,19,0,arcLenience,2,0.5,grabHeight, 0,0,0,false,'G'); //5
        Push(0);
        Push(10);
        moveCmd.MoveToPosition(speed,52,47,0,arcLenience,1,0.5,grabHeight, 0,0,0,false,'G'); //5
        moveCmd.MoveToPosition(speed,58,47,0,preciseLenience,2,0.5,grabHeight, 0,0,0,false,'G'); //6
        moveCmd.MoveToPosition(speed,58,12,0,preciseLenience,1,0.5,grabHeight, 0,0,0,false,'G'); //6

        Grab(0);
        Hook(2);

        Grab(0);
        Hook(4);

        Grab(0);
        Hook(4);

//        Grab(0);
//        Hook(2);

//        Grab(0);
//        Hook(4);
        moveCmd.MoveToPosition(speed,45,10,0,2 ,2,0.5,0,0,0,0,false,'P');

        while (true)
        {
            dt.FieldOrientedTranslate(0.3,-0.1,0,shark.GetImuReading());
        }

//        Stop();
    }

    private void Hook(int offset) // add offset constant
    {
        moveCmd.MoveToPosition(1, 0 + offset, grabDistance + 2, 0,preciseLenience,0,1,hookHeight,0, 0,0,true, 'B');
        moveCmd.MoveToPosition(0.8, 0 + offset, hookDistance, 0,0.2,1,1, hookHeight,0, 0,0,true, 'B');
//        moveCmd.MoveToPosition(0.8, 0 + offset - 4, hookDistance, 0,preciseLenience,0,0.5, 1250, true, 'B');
//            TeleopMoveCommandY(0.5, 0 + localOffset, 23.5, 0,preciseLenience,1,0.5, 1275, true, 'B');
//        odo.OdometryControl(1, offset-3, hookDistance,0,preciseLenience,0);

        dt.FieldOrientedTranslate(0,0,0,shark.GetImuReading());
        sleep(150);

        dt.FieldOrientedTranslate(-1,0,0, shark.GetImuReading());
//            s1.OdometryControl(0,0 + localOffset, 23, 0, arcLenience);
        sleep(100);
        cs.SetClawOpen(false);
        cs.Update();
        sleep(150);
        dt.FieldOrientedTranslate(0,0,0,0);
        cs.Update();
        cs.SetWristMode('S');
        sleep(150);
        moveCmd.MoveToPosition(1, 0 + offset, 14, 0, preciseLenience,1,1,hookHeight,0, 0,0,false, 'S');
    }

    private void Grab(double offset)
    {
        moveCmd.MoveToPosition(1, 40 + offset, grabDistance + 4, 0,1,2,1, grabHeight, 0, 0,0,false, 'G');
        moveCmd.MoveToPosition(0.6, 40 + offset, grabDistance, 0,preciseLenience, 2,1,grabHeight, 0, 0,0,false, 'G');
        moveCmd.MoveToPosition(0.5, 40 + offset, grabDistance, 0,arcLenience, 2,1,grabHeight, 0, 0,0,true, 'G');
//            s1.OdometryControl(0,40, grabDistance, 0,arcLenience);
        dt.FieldOrientedTranslate(0,-0.4,0,shark.GetImuReading());
        cs.SetClawOpen(true);
        sleep(200);
        moveCmd.MoveToPosition(0.5, 40 + offset, grabDistance,0,preciseLenience,1,1,hookHeight,0, 0,0,true, 'G');
        moveCmd.MoveToPosition(1, 40 + offset, grabDistance + 2, 0,preciseLenience,2,1,hookHeight,0, 0,0,true, 'B');
//        moveCmd.MoveToPosition(speed,42 + offset,grabDistance,0,0,-850,true,'G'); //8
    }

    // could try rotational push for speed
    private void Push(double offset)
    {
        moveCmd.MoveToPosition(speed,32 + offset,47,0,1,1,preciseLenience,grabHeight, 0,0,0,false,'P');
        moveCmd.MoveToPosition(speed,42 + offset,47,0,1,2,preciseLenience,grabHeight, 0,0,0,false,'P'); //6
        moveCmd.MoveToPosition(speed,42 + offset,12,0,1,1,preciseLenience,grabHeight, 0,0,0,false,'P'); //6
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
        telemetry.addData("x", shark.GetPositionX());
        telemetry.addData("y", shark.GetPositionY());
        telemetry.addData("h", shark.GetImuReading());
        telemetry.addData("ex", shark.GetErrorX());
        telemetry.addData("eh", shark.GetErrorH());
        telemetry.addData("ey", shark.GetErrorY());
        telemetry.addData("PROPORTIONAL", shark.GetPorportionalX());
        telemetry.addData("DERIVATIVE", shark.GetDerivativeX());
//        telemetry.addData("INTEGRAL X", odo.GetIntegralSumX());
//        telemetry.addData("INTEGRAL Y", odo.GetIntegralSumY());
        telemetry.update();
    }

//    private void TestSequenceUp()
//    {
////        moveCmd.MoveToPosition(speed,-24,0,-90,0,false,'/');
////        UpdateTelemetry();
////        moveCmd.MoveToPosition(speed,-12,0,-90,0,false,'/');
//        shark.TuningUp();
//        moveCmd.MoveToPosition(speed,12,0,0,0.5,2,0.5,1275,false,'/');
//        moveCmd.MoveToPosition(speed,12,12,0,0.5,2,0.5,1275,false,'/');
//        moveCmd.MoveToPosition(speed,0,0,0,0.5,2,0.5,1275,false,'/');
////        odo.OdometryControl(speed, 12, 6, 0, 0.5);
//        UpdateTelemetry();
//    }

    private void TestSequenceUpX()
    {
        shark.TuningUp();
        moveCmd.MoveToPosition(speed,0,0,0,0.5,2,1,1275,0,0,0,false,'/');
        while (true)
        {
            shark.OdometryControl(1,12,0,0,0.5,2);
            UpdateTelemetry();
        }
    }

    private void TestSequenceUpY()
    {
        shark.TuningUp();
        moveCmd.MoveToPosition(speed,0,0,0,0.5,2,1,1275,0,0,0,false,'/');
        while (true)
        {
            shark.OdometryControl(1,0,12,0,0.5,2);
            UpdateTelemetry();
        }
    }

    private void TestSequenceUpXY()
    {
        shark.TuningUp();
        moveCmd.MoveToPosition(speed,0,0,0,0.5,2,1,1275,0,0,0,false,'/');
        while (true)
        {
            shark.OdometryControl(1,12,12,0,0.5,2);
            UpdateTelemetry();
        }
    }

    private void TestSequenceDownX()
    {
        shark.TuningDown();
        shark.OdometryControl(1,12,0,0,0.5,2);
        UpdateTelemetry();
    }

    private void TestSequenceDownY()
    {
        shark.TuningDown();
        shark.OdometryControl(1,0,12,0,0.5,2);
        UpdateTelemetry();
    }

    private void TestSequenceDownXY()
    {
        shark.TuningDown();
        shark.OdometryControl(1,12,12,0,0.5,2);
        UpdateTelemetry();
    }

    private void RotationSequence()
    {
        moveCmd.MoveToPosition(0,0,0,0,10,2,1,1275,0,0,0,false,'/');
        while (true)
        {
            moveCmd.MoveToPosition(speed,0,0,180,1,2,0.5,1275,0,0,0,false,'/');
        }
    }

    // 1.5 y error BACK
    // 0.5 x error LEFT

    // 1.5 y error back
    // 0 x error left

    // 1.5 y error back
    // 0.5 x error LEFT6

    private void IntegralTest()
    {
        for (int i = 0; i < 5; i++)
        {
            moveCmd.MoveToPosition(speed,0,12,0,0.5,2,0,0,0,0,0,false,'/');
            moveCmd.MoveToPosition(speed,0,0,0,0.5,2,0,0,0,0,0,false,'/');
        }
        for (int i = 0; i < 5; i++)
        {
            moveCmd.MoveToPosition(speed,12,0,0,0.5,2,0,0,0,0,0,false,'/');
            moveCmd.MoveToPosition(speed,0,0,0,0.5,2,0,0,0,0,0,false,'/');
        }
        while (true)
        {
            shark.OdometryControl(1,0,0,0,0.5,2);
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
        shark.init(hardwareMap,true);
        cs.init(hardwareMap, true);
        am.init(hardwareMap);

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
//                NetZoneAuto();

                SpecimenSequence();
//                BasketSequence();
//                TestSequenceUp();

//                TestSequenceUpX();
//                TestSequenceUpY();
//                TestSequenceUpXY();

//                TestSequenceDownX();
//                TestSequenceDownY();
//                TestSequenceDownXY();

//                IntegralTest();
//                RotationSequence();
                // BasicPark();
            }
        }
    }
}

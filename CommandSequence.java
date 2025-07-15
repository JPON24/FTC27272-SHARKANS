package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
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

    double grabDistance = 0; // old 4.5
    double hookDistance = 34; // 22.5
    int grabHeight = 50;
    int hookHeight = 1000;

    double preciseLenience = 0.6;
    double arcLenience = 10;

    double circleYOffset = 4.75;
    int pushExtendDist = -1400;

    // push circle radius = 25.75 inches

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
        moveCmd.MoveToPosition(speed,-4,hookDistance,0,preciseLenience,2,0.5,grabHeight, 0,0,0,true,'C'); //5
        moveCmd.MoveToPosition(0,-4,hookDistance,0,arcLenience,2,0.5,hookHeight, 0,0,0,true,'C'); //5
        sleep(200);
        moveCmd.MoveToPosition(0,-4,hookDistance,0,arcLenience,2,0.5,hookHeight, 0,0,0,false,'C'); //5
        sleep(200);

        moveCmd.MoveToPosition(speed,36,24 + circleYOffset,0,preciseLenience,2,preciseLenience,grabHeight, 0,0.3,0.33,false,'G');

        Push(0);
        Push(10);

        moveCmd.MoveToPosition(speed,52,55,0,preciseLenience,2,0.5,grabHeight, 0,0,0,false,'G'); //5
        moveCmd.MoveToPosition(speed,58,55,0,preciseLenience,2,0.5,grabHeight, 0,0,0,false,'G'); //6
        moveCmd.MoveToPosition(speed,58,grabDistance+4,0,preciseLenience,1,0.5,grabHeight, 0,0,0,false,'G'); //6

        Grab(18);
        Hook(-2);

        Grab(0);
        Hook(0);

        Grab(0);
        Hook(2);

        moveCmd.MoveToPosition(speed,48,grabDistance,0,preciseLenience,2,preciseLenience,grabHeight, 0,0,0,false,'G');
    }

    private void Hook(int offset) // add offset constant
    {
        moveCmd.MoveToPosition(1, 0 + offset, hookDistance, 0,preciseLenience,2,1,grabHeight,0, 0,0,true, 'C');
        moveCmd.MoveToPosition(0, 0 + offset, hookDistance, 0,arcLenience,2,1, hookHeight,0, 0,0,true, 'C');
        sleep(200);
        moveCmd.MoveToPosition(0, 0 + offset, hookDistance, 0, arcLenience,1,1,hookHeight,0, 0,0,false, 'C');
        sleep(200);
    }

    private void Grab(double offset)
    {
        moveCmd.MoveToPosition(1, 40 + offset, grabDistance + 4, 0,1,2,1, grabHeight, 0, 0,0,false, 'G');
        moveCmd.MoveToPosition(1, 40 + offset, grabDistance, 0,preciseLenience, 2,1,grabHeight, 0, 0,0,false, 'G');
        moveCmd.MoveToPosition(0, 40 + offset, grabDistance, 0,arcLenience, 2,1,grabHeight, 0, 0,0,true, 'G');
        sleep(200);
        moveCmd.MoveToPosition(0, 40 + offset, grabDistance,0,arcLenience,1,1,200,0, 0,0,true, 'G');
//        moveCmd.MoveToPosition(speed,42 + offset,grabDistance,0,0,-850,true,'G'); //8
    }

    // could try rotational push for speed
    private void Push(double offset)
    {
        moveCmd.MoveToPosition(speed,36 + offset,24 + circleYOffset,15,1,3,preciseLenience,grabHeight, pushExtendDist,0.3,0.33,false,'G');
        moveCmd.MoveToPosition(speed,36 + offset,24 + circleYOffset,150,1,3,preciseLenience,grabHeight, pushExtendDist,0.3,0.33,false,'G'); //6
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
        telemetry.update();
    }

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

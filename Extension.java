package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Extension {
    private DcMotor extension = null;
    private Servo extendRoll = null;
    private Servo extendPitch = null;

    ElapsedTime runtime = new ElapsedTime();
//    private Servo extendYaw = null;


    int localNeutral = 0;
    boolean canUpdateLocalNeutral = true;

    int topLimit = 1500; // old value 1650
    int bottomLimit = 15;

    double localPitchPos = 0;
    double localRollPos = 0;


    public void init(HardwareMap hwMap) {
        extension = hwMap.get(DcMotor.class, "extension");
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setTargetPosition(0);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extension.setDirection(DcMotor.Direction.REVERSE);

        extendRoll = hwMap.get(Servo.class, "extendRoll");
        extendPitch = hwMap.get(Servo.class, "extendPitch");
//        extendYaw = hwMap.get(Servo.class, "extendYaw");
    }

    public void ResetEncoders() {
        topLimit = 1500;
        bottomLimit = 15;
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ResetEncodersUp() {
        topLimit = -15;
        bottomLimit = -1500;
    }

    // intakeWrist down = 0
    // intake wrist perpendicular = 0.5

    public void MoveExtend(double targetPower, char mode) {
        if (mode == 'T') {
            if (targetPower > 0.1) {
                canUpdateLocalNeutral = true;
                MoveToPosition(topLimit);
            } else if (targetPower < -0.1) {
                canUpdateLocalNeutral = true;
                MoveToPosition(bottomLimit);
            } else {
                if (canUpdateLocalNeutral) {
                    localNeutral = extension.getCurrentPosition();
                    canUpdateLocalNeutral = false;
                }
                MoveToPosition(localNeutral);
            }
        } else if (mode == 'A') {
            MoveToPosition((int) targetPower);
        }
    }

    public void SetLocalManipulatorState(double roll, double pitch) {
        localRollPos = roll;
        localPitchPos = pitch;
    }

    public void Update()
    {
        if (runtime.milliseconds() > 0.125)
        {
            runtime.reset();
            SetManipulatorState();
        }
    }

    public void SetManipulatorState()
    {
        extendRoll.setPosition(localRollPos);
        extendPitch.setPosition(localPitchPos);
//        extendYaw.setPosition(yaw);
    }

    public void MoveToPosition(int position)
    {
        extension.setTargetPosition(position);
    }
    
    public int GetCurrentPosition()
    {
        return extension.getCurrentPosition();
    }

    public boolean GetCompleted(int tgt)
    {
        // increased due to high range
        int lenience = 30;
        int error = Math.abs(tgt - extension.getCurrentPosition());
        return error < lenience;
    }
}

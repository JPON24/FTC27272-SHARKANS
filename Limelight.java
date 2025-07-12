package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class Limelight {
    Limelight3A limelight;

    SparkFunOTOS.Pose2D lastPosition = new SparkFunOTOS.Pose2D();
    boolean isValid;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(15); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
    }

    public SparkFunOTOS.Pose2D GetLimelightData(boolean redAlliance, double orientation)
    {
        limelight.updateRobotOrientation(orientation);

        LLResult result = limelight.getLatestResult();
        Pose3D pose = result.getBotpose();

        SparkFunOTOS.Pose2D output = new SparkFunOTOS.Pose2D(0,0,0);

        if (!result.isValid() || pose == null) {
            isValid = false;
            return output;
        }
        else
        {
            isValid = true;
        }

        output.x = pose.getPosition().x;
        output.y = pose.getPosition().y;

        output = ProcessCoordinates(redAlliance, output);

        lastPosition = output;

        return output;
    }

    private SparkFunOTOS.Pose2D ProcessCoordinates(boolean redAlliance, SparkFunOTOS.Pose2D pos)
    {
        SparkFunOTOS.Pose2D processedPosition = new SparkFunOTOS.Pose2D();
        processedPosition.x = MtoIn(pos.x);
//        processedPosition.y = 2 - (pos.y/Math.abs(pos.y));
        processedPosition.y = 2 - Math.abs(pos.y);

        processedPosition.y = MtoIn(processedPosition.y);

        if (!redAlliance)
        {
            processedPosition.y *= -1;
            processedPosition.x *= -1;
        }

        return processedPosition;
    }

    private double MtoIn(double input)
    {
        return input / 0.0254;
    }

    public boolean GetIsValid()
    {
        return isValid;
    }

    public SparkFunOTOS.Pose2D GetLastPosition()
    {
        return lastPosition;
    }
}

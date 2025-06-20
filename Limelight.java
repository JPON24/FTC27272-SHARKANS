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

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(30); // This sets how often we ask Limelight for data (100 times per second)
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
            return output;
        }

        output.x = pose.getPosition().x;
        output.y = pose.getPosition().y;

        if (redAlliance)
        {

        }

        return output;
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class ActionHandler {
    static int actionId;
    
    public void IncrementActionId()
    {
        actionId += 1;
    }
    
    public void SetActionId(int newId)
    {
        actionId = newId;
    }
    
    public int GetActionId()
    {
        return actionId;
    }
}

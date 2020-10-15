package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto" ,group = "Auto")

public class Auto extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        
    }
}

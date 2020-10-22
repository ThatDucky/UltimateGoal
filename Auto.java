package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Auto" ,group = "Auto")

public class Auto extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        double revs = (28/(10 * 3.14));

        robot.init(hardwareMap);
        waitForStart();

        robot.setTargetPosition((int)revs);
        robot.one.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setPower(1);
        while(robot.one.isBusy()){
            sleep(10);
        }
        robot.setPower(0);
        robot.reset();
    }
}

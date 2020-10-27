package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Auto" ,group = "Auto")

public class Auto extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        goToPosition(10,0.8);
    }

    public void goToPosition(int decimeters, double power){
        robot.setTargetPosition(decimeters);
        robot.setMode(1);
        robot.setPower(power, power);
        while(robot.one.isBusy() || robot.two.isBusy() || robot.three.isBusy() || robot.four.isBusy()){
            sleep(10);
        }
        robot.setPower(0,0);
        robot.setMode(0);
    }
}

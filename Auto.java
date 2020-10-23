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

        robot.setMode(2);
        robot.one.setPower(1);
        robot.two.setPower(1);
        robot.three.setPower(1);
        robot.four.setPower(1);

        //robot.setPower(1,1);
        sleep(10000);
        //goToPosition(9,10);
    }

    public void goToPosition(int decimeters, double power){
        robot.setMode(0);
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

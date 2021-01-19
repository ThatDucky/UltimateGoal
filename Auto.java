package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.Math;

@Autonomous(name = "Auto", group = "Auto")

public class Auto extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
            robot.init(hardwareMap);
            telemetry.addData("Status: ", "Ready");
            telemetry.update(); //setup telemetry and call it
            waitForStart();

            robot.setPower(0.20, 0.20);
            while(robot.color.blue() < 60){
                sleep(100);
            }
            robot.setPower(0, 0);
            robot.fWheelPower(robot.powerShot);
            sleep(3000);
            robot.launcher.setPosition(robot.fire);
            sleep(1000);
            robot.launcher.setPosition(robot.rest);
            robot.fWheelPower(0);
    }

    public void goToPosition(int decimeters, double power){
        //go through the steps to get to target distance
        decimeters *= -1;
        robot.setTargetPosition(decimeters);
        telemetry.addData("Running To Position", robot.getTargetPosition());
        telemetry.update();
        robot.setMode(1);
        robot.setPower(power, power);
        while(robot.isBusy()){
            sleep(10);
        }
        robot.setPower(0,0);
        telemetry.addData("Running To Position", "Done");
        telemetry.update();
        robot.setMode(0);
    }
}

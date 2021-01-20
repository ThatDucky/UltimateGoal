package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.lang.Math;

@Autonomous(name = "Auto", group = "Auto")

public class Auto extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
            robot.init(hardwareMap);
            robot.setMode(2);
            while(!robot.imu.isGyroCalibrated()) {
                telemetry.addData("Gyro: ", "Calibrating...");
                telemetry.update();
            }
            float homeX = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
            float homeY = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;

            telemetry.addData("Status: ", "Ready");
            telemetry.update(); //setup telemetry and call it
            waitForStart();

            robot.setPower(0.15, 0.15);
            while(robot.color.blue() < 60){
                sleep(10);
            }
            robot.setPower(0, 0);
            goToPosition(-1,0.2);
            robot.fWheelPower(robot.powerShot);
            while(robot.fWheelOne.getVelocity() < 2060 || robot.fWheelTwo.getVelocity() < 2060){
                sleep(100);
            }
            turnHome(homeX);
            robot.launcher.setPosition(robot.fire);
            sleep(800);
            robot.launcher.setPosition(robot.rest);
            robot.fWheelPower(0);
            sleep(1000);
    }

    public void turnHome(float homeX){
        while(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle != homeX){
            robot.setPower(0.20,-0.20);
            sleep(100);
        }
        while(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle != homeX){
            robot.setPower(-0.15,0.15);
            sleep(100);
        }
        robot.setPower(0,0);
    }

    public void goToPosition(int decimeters, double power){
        //go through the steps to get to target distance
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

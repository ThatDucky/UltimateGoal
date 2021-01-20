package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;

@Autonomous(name = "AutoTest", group = "Auto")

public class AutoTest extends LinearOpMode{
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);


        while(!robot.imu.isGyroCalibrated()){
            telemetry.addData("Gyro: ", "Calibrating");
            telemetry.update();
        }
        telemetry.addData("Gyro: ", "Ready");
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Gyro: ", "" + robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle + " " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
            telemetry.update();
        }
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

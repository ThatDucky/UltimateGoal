package org.firstinspires.ftc.teamcode;

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
        robot.setMode(0); //sets motors to reset
        double tPower = 0.13;
        float home = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        telemetry.addData("Status: ", "Ready");
        telemetry.update(); //setup telemetry and call it
        waitForStart();

        goToLine();
        turnTo(home, tPower);
        goToPosition(-1,0.25);
        fire(robot.powerShot);
        sleep(1000);
        turnTo(-4, tPower);
        sleep(500);
        fire(robot.powerShot);
        sleep(1000);
        turnTo(-8, tPower);
        sleep(500);
        fire(robot.powerShot);
        sleep(1000);
        turnTo(home, tPower);
        sleep(500);
        goToPosition(-10,0.40);
    }

    public void fire(double power){
        //spins up the fly wheel and fires the servo then resets everything
        robot.fWheelPower(power);
        double velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity
        while(velocity < power){
            velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity update
            sleep(100);
        }
        sleep(500);
        robot.launcher.setPosition(robot.fire);
        sleep(800);
        robot.launcher.setPosition(robot.rest);
        robot.fWheelPower(0);
    }

    public void goToLine(){
        //moves forward until the color sensor fine the line
        robot.setMode(2);
        robot.setPower(0.15, 0.15);
        while(robot.color.blue() < 60){
            sleep(10);
        }
        robot.setPower(0, 0);
    }

    public void turnTo(float point, double power){
        //rotates the robot until the gyro fines the defined point then checks a few times
        robot.setMode(2);
        for(int i = 0; i < 4; i++){
            if(point > robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle){
                while ((point - 2) > robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) {
                    robot.setPower(power * -1, power);
                    sleep(10);
                }
            }else if(point < robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle){
                while ((point + 2) < robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) {
                    robot.setPower(power, power * -1);
                    sleep(10);
                }
            }
        }
        robot.setPower(0,0);
        robot.setMode(0);
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

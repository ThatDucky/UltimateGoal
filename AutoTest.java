package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.lang.Math;

@Autonomous(name = "AutoTest", group = "Auto")

public class AutoTest extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.setMode(2);
        telemetry.addData("Gyro: ", "Ready");
        telemetry.update();
        double tPower = 0.13;
        float home = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        waitForStart();

        robot.pattern = robot.pattern.next();
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Blinkin: ", robot.pattern.toString());
        telemetry.update();
        sleep(3000);
        robot.pattern = robot.pattern.next();
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Blinkin: ", robot.pattern.toString());
        telemetry.update();
        sleep(3000);
        robot.pattern = robot.pattern.next();
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Blinkin: ", robot.pattern.toString());
        telemetry.update();
        sleep(3000);
        robot.pattern = robot.pattern.next();
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Blinkin: ", robot.pattern.toString());
        telemetry.update();
        sleep(3000);
        robot.pattern = robot.pattern.next();
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Blinkin: ", robot.pattern.toString());
        telemetry.update();
        sleep(5000);
    }

    public void turnTo(float point, double power){
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

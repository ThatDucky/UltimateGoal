package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.lang.Math;
@Disabled
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

        armToPosition(0);
        armToPosition(0);
        sleep(3000);
        armToPosition(1);
        armToPosition(1);
    }

    public void armToPosition(int pos){
        if(pos == 1){
            robot.arm.setPower(0.25);
            sleep(2500);
            robot.arm.setPower(0);
        }else{
            robot.arm.setPower(-0.25);
            sleep(2500);
            robot.arm.setPower(0);
        }
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

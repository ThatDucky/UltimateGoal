package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "AutoRedDefault", group = "Auto")

public class AutoRedDefault extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.setMode(0); //sets motors to reset
        robot.claw.setPosition(robot.closed);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        float home = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        telemetry.addData("Status: ", "Ready");
        telemetry.update(); //setup telemetry and call it
        waitForStart();

        goToLine(0.20);
        turnTo(-90, 0.35);
        goToPosition(2, 0.35);
        turnTo(home,0.13);
        goToPosition(0.5, 0.35);
        armToPosition(0);
        robot.claw.setPosition(robot.open);
        goToPosition(-3,0.35);
        armToPosition(2);
        robot.claw.setPosition(robot.closed);
        turnTo(home,0.13);
        fire(robot.highGoal);
        goToPosition(1.5,0.5);
        robot.shove.setPosition(robot.shoved);
    }

    public void armToPosition(int pos){
        if(pos == 1){
            robot.arm.setPower(0.25);
        }else if(pos == 2){
            robot.arm.setPower(0.17);
        }else{
            robot.arm.setPower(-0.25);
        }
        sleep(2500);
        robot.arm.setPower(0);
    }

    public void fire(double power){
        //spins up the fly wheel and fires the servo then resets everything
        robot.fWheelPower(power);
        double velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity
        while(velocity < power){
            velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity update
            sleep(100);
        }
        while(velocity > (power + 10)){
            velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity update
            sleep(100);
        }
        sleep(250);
        robot.launcher.setPosition(robot.fire);
        sleep(1000);
        robot.launcher.setPosition(robot.rest);
        sleep(1000);
        robot.launcher.setPosition(robot.fire);
        sleep(1000);
        robot.launcher.setPosition(robot.rest);
        sleep(1000);
        robot.launcher.setPosition(robot.fire);
        sleep(1000);
        robot.launcher.setPosition(robot.rest);
        robot.fWheelPower(0);
    }

    public void goToLine(double power){
        //moves forward until the color sensor fine the white line
        robot.setMode(2);
        robot.setPower(power, power);
        while(robot.color.blue() < 15 && robot.color.green() < 15 && robot.color.red() < 15){
            sleep(10);
        }
        robot.setPower(0,0);
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

    public void goToPosition(double decimeters, double power){
        //go through the steps to get to target distance
        robot.setTargetPosition(decimeters);
        telemetry.addData("Running To Position", robot.getTargetPosition());
        telemetry.update();
        robot.setMode(1);
        robot.setPower(power, power);
        while(robot.one.isBusy()){
            sleep(10);
        }
        robot.setPower(0,0);
        telemetry.addData("Running To Position", "Done");
        telemetry.update();
    }
}

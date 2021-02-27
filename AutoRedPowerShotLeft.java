package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "AutoRedPowerShotLeft", group = "Auto")

public class AutoRedPowerShotLeft extends LinearOpMode {
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
        turnTo(home, 0.15);
        goToPosition(-1.5,0.15);
        turnTo(home,0.15);
        fire(robot.powerShot - 105);
        turnTo(4,0.15);
        fire(robot.powerShot - 115);
        turnTo(10,0.15);
        fire(robot.powerShot - 125);
        robot.fWheelPower(0);
        turnTo(-25,0.25);
        goToPosition(3.0,0.20);
        armToPosition(0);
        robot.shove.setPosition(robot.shoved);
        robot.claw.setPosition(robot.open);
        goToPosition(-1,0.25);
        sleep(1000);
    }

    public void armToPosition(int pos){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        if(pos == 1){
            //up
            robot.arm.setTargetPosition(0);
        }else if(pos == 2){
            //half up
            robot.arm.setTargetPosition((int)((28 / (28 * 3.14)) * 125) * -11);
        }else{
            //down
            robot.arm.setTargetPosition((int)((28 / (28 * 3.14)) * 125) * -22);
        }
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.50);
    }

    public void fire(double power){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        //spins up the fly wheel and fires the servo then resets everything
        robot.fWheelPower(power);
        double velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity
        for(int i = 0; i < 2; i++){
            while(velocity < (power - 10)){
                velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity update
                sleep(100);
            }
            while(velocity > (power + 10)){
                velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity update
                sleep(100);
            }
        }
        robot.launcher.setPosition(robot.fire);
        sleep(1000);
        robot.launcher.setPosition(robot.rest);
        sleep(250);
    }

    public void goToLine(double power){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        //moves forward until the color sensor fine the white line
        robot.setMode(2);
        robot.setPower(power, power);
        while(robot.color.blue() < 25 && robot.color.green() < 25 && robot.color.red() < 25){
            sleep(10);
        }
        robot.setPower(0,0);
    }

    public void turnTo(float point, double power){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        //rotates the robot until the gyro fines the defined point then checks a few times
        robot.setMode(2);
        for(int i = 0; i < 4; i++){
            if(point > robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle){
                while ((point - 1) > robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) {
                    robot.setPower(power * -1, power);
                    sleep(10);
                }
            }else if(point < robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle){
                while ((point + 1) < robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) {
                    robot.setPower(power, power * -1);
                    sleep(10);
                }
            }
        }
        robot.setPower(0,0);
        robot.setMode(0);
    }

    public void goToPosition(double decimeters, double power){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
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

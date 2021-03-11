package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.Math;

//@Disabled
@Autonomous(name = "AutoTest", group = "Auto")

public class AutoTest extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.setMode(2);
        telemetry.addData("Gyro: ", "Ready");
        telemetry.update();
        double ground = robot.dis.getDistance(DistanceUnit.CM);
        float home = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        waitForStart();

        goToPosition(-2,0.10);
        sleep(1500);
        int ring = ringScan(ground);
        telemetry.addData("Rings: ", "" + ring);
        telemetry.update();
        if(ring > 2){
            goToLine(0.15);
            turnTo(15,0.25);
            goToPosition(5.5,0.25);
            armToPosition(0);
            sleep(1500);
            robot.shove.setPosition(robot.shoved);
        }else if(ring > 1 && ring <= 2){
            goToLine(0.15);
            turnTo(home,0.25);
            goToPosition(2,0.25);
            armToPosition(0);
            sleep(1500);
            robot.shove.setPosition(robot.shoved);
        }else{
            goToLine(0.15);
            turnTo(80,0.25);
            goToPosition(2,0.25);
            armToPosition(0);
            sleep(1500);
            robot.shove.setPosition(robot.shoved);
        }
        sleep(1000);
    }

    public void armToPosition(int pos){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Moving Arm: ", "In Progress");
        telemetry.update();
        //moves the arm to one of three options. up, down, half
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
        robot.arm.setPower(0.80);
        
    }

    public void fire(double power){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Shooting: ", "In Progress");
        telemetry.update();
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
        telemetry.addData("Shooting: ", "Done");
        telemetry.update();
    }

    public void goToLine(double power){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Finding Line: ", "In Progress");
        telemetry.update();
        //moves forward until the color sensor fine the white line
        robot.setMode(2);
        robot.setPower(power, power);
        while(robot.color.blue() < 25 && robot.color.green() < 25 && robot.color.red() < 25){
            sleep(10);
        }
        robot.setPower(0,0);
        telemetry.addData("Finding Line: ", "Done");
        telemetry.update();
    }

    public void turnTo(float point, double power){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Turning: ", "In Progress");
        telemetry.update();
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
        telemetry.addData("Turning: ", "Done");
        telemetry.update();
    }

    public void goToPosition(double decimeters, double power){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Running To Position: ", "In Progress");
        telemetry.update();
        //go through the steps to get to target distance
        robot.setTargetPosition(decimeters);
        robot.setMode(1);
        robot.setPower(power, power);
        while(robot.one.isBusy()){
            sleep(10);
        }
        robot.setPower(0,0);
        telemetry.addData("Running To Position: ", "Done");
        telemetry.update();
    }

    public int ringScan(double ground){
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        telemetry.addData("Scanning For Rings: ", "In Progress");
        telemetry.update();
        //finds the difference between current distance and ground then divides it by the ring distance.
        double scan = robot.dis.getDistance(DistanceUnit.CM);
        double dif = (ground - scan);
        telemetry.addData("Scanning For Rings: ", "Done");
        telemetry.update();
        return (int)Math.round(dif / 2.2); //Thickness of the ring ~2.2 cm
    }
}

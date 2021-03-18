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
    //calls the hardware class

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        //initialize hardware
        robot.setMode(0);
        //reset encoders
        robot.claw.setPosition(robot.closed);
        //makes the claw hold the wobble goal
        telemetry.addData("Auto: ", "Ready");
        telemetry.update();
        //setup display telemetry
        double ground = robot.dis.getDistance(DistanceUnit.CM);
        //start distance from the ground
        float home = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        //start home position
        waitForStart();
        /*
        goToLine(0.25);
        turnTo(home, 0.20);
        goToPosition(-1.5,0.20,false);
        turnTo(home,0.20);
         */
        double angle = Math.abs(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("V: ","" + robot.calculateVelocity((1.65 / Math.cos(angle)),0.77));
        telemetry.update();
        sleep(5000);
        fire(0.77);
        turnTo(4,0.20);
        fire(0.77);
        turnTo(10,0.20);
        fire(0.77);
        //fires at Power Shots
        robot.fWheelPower(0);
        /*
        //powers off the flywheel
        turnTo(30,0.30);
        goToPosition(-5.5,0.25,false);
        int ring = ringScan(ground);
        turnTo(home, 0.20);
        if(ring > 2){
            goToLine(0.25);
            turnTo(-20,0.30);
            goToPosition(8,0.30, true);
            armToPosition(0);
            robot.claw.setPosition(robot.open);
            sleep(500);
            goToPosition(-6,0.30,false);
        }else if(ring >= 1 && ring <= 2){
            goToLine(0.25);
            turnTo(home,0.30);
            goToPosition(1.5,0.30, true);
            armToPosition(0);
            robot.claw.setPosition(robot.open);
            sleep(500);
            goToPosition(-2,0.30,false);
        }else{
            goToLine(0.25);
            turnTo(-70,0.30);
            goToPosition(1.5,0.30,true);
            armToPosition(0);
            robot.claw.setPosition(robot.open);
            sleep(500);
            goToPosition(-2,0.30,false);
        }
         */
    }

    public void armToPosition(int pos){
        //lights for the action stated
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        //adds the display telemetry for the action stated
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
            robot.arm.setTargetPosition((int)((28 / (28 * 3.14)) * 125) * -33);
        }
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1.00);
        while(robot.arm.isBusy()){
            sleep(100);
        }
        telemetry.addData("Moving Arm: ", "Done");
        telemetry.update();
    }

    public void fire(double shotH){
        //lights for the action stated
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        //adds the display telemetry for the action stated
        telemetry.addData("Shooting: ", "In Progress");
        telemetry.update();
        //spins up the fly wheel and fires the servo then resets everything
        double angle = Math.abs(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        double dy = shotH;
        double dx = 1.65 / Math.cos(angle);
        double power = robot.calculateVelocity(dx,dy);
        //calculates the offset for the power shot goal
        robot.fWheelPower(power);
        double velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity
        for(int i = 0; i < 2; i++){
            while(velocity < (power - 7.5)){
                velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity update
                sleep(100);
            }
            while(velocity > (power + 7.5)){
                velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity update
                sleep(100);
            }
        }
        robot.launcher.setPosition(robot.fire);
        sleep(750);
        robot.launcher.setPosition(robot.rest);
        sleep(250);
        telemetry.addData("Shooting: ", "Done");
        telemetry.update();
    }

    public void goToLine(double power){
        //lights for the action stated
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        //adds the display telemetry for the action stated
        telemetry.addData("Finding Line: ", "In Progress");
        telemetry.update();
        //moves forward until the color sensor fine the white line
        robot.setMode(2);
        robot.setPower(power, power);
        while(robot.color.blue() < 25 && robot.color.green() < 25 && robot.color.red() < 25){
            sleep(10);
        }
        robot.setPower(0,0);
        //reset motors to 0
        telemetry.addData("Finding Line: ", "Done");
        telemetry.update();
    }

    public void turnTo(float point, double power){
        //lights for the action stated
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        //adds the display telemetry for the action stated
        telemetry.addData("Turning: ", "In Progress");
        telemetry.update();
        //rotates the robot until the gyro fines the defined point then checks a few times
        robot.setMode(2);
        for(int i = 0; i < 2; i++){
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
            power *= 0.75;
            if(power < 0.08){
                power = 0.08;
            }
        }
        robot.setPower(0,0);
        robot.setMode(0);
        //reset motors to 0
        telemetry.addData("Turning: ", "Done");
        telemetry.update();
    }

    public void goToPosition(double decimeters, double power, boolean absolutePosition){
        //lights for the action stated
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        //adds the display telemetry for the action stated
        telemetry.addData("Running To Position: ", "In Progress");
        telemetry.addData("Absolute Position: ", "" + absolutePosition);
        telemetry.update();
        //go through the steps to get to target distance
        robot.setTargetPosition(decimeters);
        robot.setMode(1);
        robot.setPower(power, power);
        while(robot.isBusy() && (!robot.atTarget() || absolutePosition)){
            sleep(10);
        }
        robot.setPower(0,0);
        //reset motors the 0
        telemetry.addData("Running To Position: ", "Done");
        telemetry.addData("Absolute Position: ", "" + absolutePosition);
        telemetry.update();
    }

    public int ringScan(double ground){
        //lights for the action stated
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
        //adds the display telemetry for the action stated
        telemetry.addData("Scanning For Rings: ", "In Progress");
        telemetry.update();
        //finds the difference between current distance and ground then divides it by the ring distance.
        double scan = robot.dis.getDistance(DistanceUnit.CM);
        double dif = (ground - scan);
        telemetry.addData("Scanning For Rings: ", "Done");
        telemetry.update();
        return (int)Math.round(dif / 2);
        //Thickness of the ring ~2 cm
    }
}

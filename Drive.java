package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;

import java.lang.Math;

@TeleOp(name = "Drive", group = "Drive")

public class Drive extends OpMode {
    Hardware robot = new Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.setMode(2); //set to run using encoders
        robot.gyro.calibrate();
        while(robot.gyro.isCalibrating()){
            telemetry.addData("Gyro: ", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("Gyro: ","Ready");
        telemetry.addData("Status: ", "Ready");
        telemetry.update(); //setup telemetry and call it
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double deadZone = 0.10; //controller dead zone
        double velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity
        double xOffSet = 0.70; //x off set for movement

        //gets x and y values of the game pad and offsets the y value by a percent of the x
        double left = (gamepad1.left_stick_y * -1) + (gamepad1.left_stick_x * xOffSet);
        double right = (gamepad1.left_stick_y * -1) - (gamepad1.left_stick_x * xOffSet);

        if(left > deadZone || left < (deadZone * -1) || right > deadZone || right < (deadZone * -1)){
            //passes power to the motor if the game pad is pushed farther than the dead zone in any direction
            robot.setPower(left, right);
        }else{
            //kills power otherwise
            robot.setPower(0,0);
        }


        if(gamepad1.left_trigger > 0){
            //set Fly Wheel To Spin Up if Left Trigger Is Held
            robot.fWheelPower(robot.highGoal);
        }else if(gamepad1.left_bumper){
            //sets the fly wheel speed to the power shot goal if bummer is held
            robot.fWheelPower(robot.powerShot);
        }else{
            //Reset fly wheel if left Trigger is not pressed
            robot.fWheelPower(0);
        }

        if(gamepad1.right_trigger > 0 && velocity >= 2000){
            //sets the  servo to fire
            robot.launcher.setPosition(robot.fire);
        }else{
            //resets the servo
            robot.launcher.setPosition(robot.rest);
        }

        //set up the display telemetry
        telemetry.addData("Left Stick Position: ", gamepad1.left_stick_x + " " + gamepad1.left_stick_y);
        telemetry.addData("Velocity: ", "" + velocity);
        telemetry.addData("Blue: ", "" + robot.color.blue());
        telemetry.addData("Gyro: ", "X:" + robot.gyro.rawX() + " Y:" + robot.gyro.rawY() + " Z:" + robot.gyro.rawZ());
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.Math;

@TeleOp(name = "Drive", group = "Drive")

public class Drive extends OpMode {
    Hardware robot = new Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.setMode(2); //set to run without encoders
        telemetry.addData("Status", "Ready");
        telemetry.update(); //setup telemetry and call it
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double deadZone = 0.12;
        telemetry.addData("Left Stick Position", gamepad1.left_stick_x + " " + gamepad1.left_stick_y);
        telemetry.update();

        if(gamepad1.left_stick_y > deadZone || gamepad1.left_stick_y < (deadZone * -1)){
                //set both sides to positive/negative power for going forward or backward
                robot.setPower(gamepad1.left_stick_y, gamepad1.left_stick_y);
        }else if(gamepad1.left_stick_x > deadZone || gamepad1.left_stick_x < (deadZone * -1)){
                 //left side gets negative power for turning
                robot.setPower((gamepad1.left_stick_x * -1), gamepad1.left_stick_x);
        }else{
                //if no joystick input reset to 0 power
                robot.setPower(0,0);
        }
        /*
        if(gamepad1.left_trigger > 0){
            //set Fly Wheel To Spin Up if Left Trigger Is Held
            robot.fWheelPower(1);
            if(gamepad1.right_trigger > 0){
                //Set Launcher To Fire if Right Trigger is Pressed
                robot.launcher.setPosition(robot.fire);
            }else{
                //Reset Launcher To Rest Posistion if Left Trigger is Not Pressed
                robot.launcher.setPosition(robot.rest);
            }
        }else{
            //Reset everything if left Trigger is not pressed
            robot.fWheelPower(0);
            robot.launcher.setPosition(robot.rest);
        }
        */
    }
}
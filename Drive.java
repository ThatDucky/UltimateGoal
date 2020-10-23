package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Drive", group = "Drive")

public class Drive extends OpMode {
        Hardware robot = new Hardware();

        @Override
        public void init() {

        }

        @Override
        public void init_loop() {

        }

        @Override
        public void start() {

        }

        @Override
        public void loop() {
                robot.init(hardwareMap); //initialize robot
                robot.setMode(2); //set to run without encoders
                double deadZone = 0.12;


                if(gamepad1.left_stick_y > deadZone || gamepad1.left_stick_y < (deadZone * -1)){
                        //set both sides to positive/negative power for going forward or backward
                        robot.setPower(gamepad1.left_stick_y, gamepad1.left_stick_y);
                }else if(gamepad1.left_stick_x > deadZone || gamepad1.left_stick_x < (deadZone * -1)){
                        //left side gets negative power for turning
                        robot.setPower((gamepad1.left_stick_x * -1), gamepad1.left_stick_x);
                }
        }
}

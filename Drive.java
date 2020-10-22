package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.Math;

@TeleOp(name = "Drive", group = "TeleOp")

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
                robot.init(hardwareMap);
                robot.one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double deadzone = 0.1;
                if(gamepad1.left_stick_y > deadzone || gamepad1.left_stick_y < (deadzone * -1)){
                        robot.one.setPower(gamepad1.left_stick_y);
                }
        }
}

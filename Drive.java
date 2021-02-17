package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.lang.Math;

@TeleOp(name = "Drive", group = "Drive")

public class Drive extends OpMode {
    Hardware robot = new Hardware();

    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.setMode(2); //set to run using encoders
        //rev color
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
        robot.revBlinkinLedDriver.setPattern(robot.pattern);
    }

    @Override
    public void start(){
        telemetry.addData("Status: ", "Ready");
        telemetry.update(); //setup telemetry and call it
    }

    @Override
    public void loop(){
        double deadZone = 0.13; //controller dead zone
        double velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity
        double xOffSet = 0.45; //x off set for movement

        //lights default color
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;

        //gets x and y values of the game pad and offsets the y value by a percent of the x
        double left = (gamepad1.left_stick_y * -1) + (gamepad1.left_stick_x * xOffSet);
        double right = (gamepad1.left_stick_y * -1) - (gamepad1.left_stick_x * xOffSet);

        if(left > deadZone || left < (deadZone * -1) || right > deadZone || right < (deadZone * -1)){
            //passes power to the motor if the game pad is pushed farther than the dead zone in any direction
            robot.setPower(left, right);
        }else{
            //kills power otherwise
            robot.setPower(0, 0);
        }

        if(gamepad1.right_bumper){
            //spins the motor to pick up rings
            robot.lift.setPower(-1.00);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        }else if(gamepad1.b){
            //reverse the motor to unstuck rings
            robot.lift.setPower(1.00);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        }else{
            //kills otherwise
            robot.lift.setPower(0);
        }

        if(gamepad1.right_stick_y > deadZone || gamepad1.right_stick_y < (deadZone * -1)){
            //passes power to the motor if the game pad is pushed farther than the dead zone
            robot.arm.setPower(gamepad1.right_stick_y * -0.25);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
        }else{
            //kills power otherwise
            robot.arm.setPower(0);
        }

        if(gamepad1.right_stick_x > deadZone){
            robot.claw.setPosition(robot.closed);
        }else if(gamepad1.right_stick_x < (deadZone * -1)){
            robot.claw.setPosition(robot.open);
        }

        if(gamepad1.left_trigger > 0){
            //set Fly Wheel To Spin Up if Left Trigger Is Held
            robot.fWheelPower(robot.highGoal);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }else if(gamepad1.left_bumper){
            //sets the fly wheel speed to the power shot goal if bummer is held
            robot.fWheelPower(robot.powerShot);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        }else{
            //Reset fly wheel if left Trigger is not pressed
            robot.fWheelPower(0);
        }

        if(gamepad1.right_trigger > 0 && velocity >= (robot.powerShot - 15) && velocity <= (robot.highGoal + 15)){
            //sets the  servo to fire
            robot.launcher.setPosition(robot.fire);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
        }else if(gamepad1.right_trigger > 0 && velocity <= 100.0){
            //sets the  servo to fire
            robot.launcher.setPosition(robot.fire);
        }else{
            //resets the servo
            robot.launcher.setPosition(robot.rest);
        }

        if(gamepad1.x){
            //push/shove the ramp
            robot.shove.setPosition(robot.shoved);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        }else{
            //otherwise rest the servo
            robot.shove.setPosition(robot.lay);
        }

        //Rev lights
        robot.revBlinkinLedDriver.setPattern(robot.pattern);

        //set up the display telemetry
        telemetry.addData("Left Stick Position: ", gamepad1.left_stick_x + " " + gamepad1.left_stick_y);
        telemetry.addData("Velocity: ", "" + velocity);
        telemetry.addData("Green: ", "" + robot.color.green());
        telemetry.addData("Gyro: ", "" + robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("LED: ", robot.pattern.toString());
        telemetry.update();//call the display telemetry
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.Math;

//@Disabled
@TeleOp(name = "DriveTest", group = "Drive")

public class DriveTest extends OpMode {
    Hardware robot = new Hardware();
    //calls the hardware class

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
        telemetry.addData("Drive: ", "Ready");
        telemetry.update(); //setup telemetry and call it
    }

    @Override
    public void loop(){
        double deadZone = 0.13; //controller dead zone
        double velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity
        double xOffSet = 0.525; //x off set for movement
        double angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double shot = 0.0;
        double dx = 1.65 / Math.cos(angle);
        double dy = 0.0;

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

        if(gamepad1.right_stick_y > (deadZone + 0.1) || gamepad1.right_stick_y < ((deadZone + 0.1) * -1)){
            //passes power to the motor if the game pad is pushed farther than the dead zone
            robot.arm.setPower(gamepad1.right_stick_y * -0.25);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
        }else{
            //kills power otherwise
            robot.arm.setPower(0);
        }

        if(gamepad1.right_stick_x > (deadZone + 0.35)){
            robot.claw.setPosition(robot.closed);
        }else if(gamepad1.right_stick_x < ((deadZone + 0.35) * -1)){
            robot.claw.setPosition(robot.open);
        }

        if(gamepad1.left_trigger > 0){
            //set Fly Wheel To Spin Up if Left Trigger Is Held
            dy = 0.90;
            shot = robot.calculateVelocity(dx,dy);
            robot.fWheelPower(shot);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }else if(gamepad1.left_bumper){
            //sets the fly wheel speed to the power shot goal if bummer is held
            dy = 0.77;
            shot = robot.calculateVelocity(dx,dy);
            robot.fWheelPower(shot);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        }else{
            //Reset fly wheel if left Trigger is not pressed
            robot.fWheelPower(0);
        }

        if(gamepad1.right_trigger > 0){
            if(velocity <= 100.0 || (velocity >= (shot - 100) && velocity <= (shot + 100))){
                //sets the  servo to fire
                robot.launcher.setPosition(robot.fire);
                //rev color
                robot.pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
            }
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
        //telemetry.addData("Gyro: ", "" + robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        //telemetry.addData("LED: ", robot.pattern.toString());
        //telemetry.addData("Distance: ",""+ robot.dis.getDistance(DistanceUnit.CM));
        telemetry.update();//call the display telemetry
    }
}

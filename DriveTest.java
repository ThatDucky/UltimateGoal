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

@Disabled
@TeleOp(name = "DriveTest", group = "Drive")

public class DriveTest extends OpMode {
    Hardware robot = new Hardware();
    //calls the hardware class
    public float home = 0;
    //sets the home value to be used for velocity

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
        double xOffSet = 0.70; //x off set for turning movement
        double zoomDis = robot.zoom.getDistance(DistanceUnit.METER);
        double v;

        //lights default color
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;

        if(!(robot.imu.isGyroCalibrated())){
            //set display while imu is resetting
            telemetry.addData("IMU Is Calibrating ", "Do Not Move The Robot");
            //changes the lights while the imu is resetting
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE;
        }

        if(gamepad1.y){
            //resets the imu
            robot.resetImu();
        }

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
            double angle = Math.abs(home - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            double dy = 0.90;
            double dx = (3.58 - ((zoomDis * Math.cos(angle)) + 0.25)) / Math.cos(angle);
            v = robot.calculateVelocity(dx,dy);
            robot.fWheelPower(v);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }else if(gamepad1.left_bumper){
            //sets the fly wheel speed to the power shot goal if bummer is held
            double angle = Math.abs(home - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            double dy = 0.77;
            double dx = (3.58 - ((zoomDis * Math.cos(angle)) + 0.25)) / Math.cos(angle);
            v = robot.calculateVelocity(dx,dy);
            robot.fWheelPower(v);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        }else{
            //Reset fly wheel if left Trigger is not pressed
            robot.fWheelPower(0);
            v = 0;
        }

        if(gamepad1.right_trigger > 0){
            if(velocity >= (v - 10) && velocity <= (v + 10)){
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
        telemetry.addData("Velocity: ", velocity);
        telemetry.addData("2M Distance: ", zoomDis);
        telemetry.addData("Velocity Target: ", v);
        //telemetry.addData("Gyro: ", "" + robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        //telemetry.addData("LED: ", robot.pattern.toString());
        telemetry.addData("Distance: ",(3.58 - ((zoomDis * Math.cos(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle)) + 0.25)) / Math.cos(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle));
        telemetry.update();//call the display telemetry
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.lang.Math;

import static android.os.SystemClock.sleep;

@Disabled
@TeleOp(name = "DriveTest", group = "Drive")

public class DriveTest extends OpMode {
    Hardware robot = new Hardware();
    public int loop = 0;
    public float home = 0;

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
        if(loop == 0){
            home = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle; //z-axis
            loop++;
        }
        double deadZone = 0.13; //controller dead zone
        double velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity
        double xOffSet = 0.45; //x off set for movement

        //lights default color
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;

        //gets x and y values of the game pad and offsets the y value by a percent of the x
        double left = (gamepad1.left_stick_y * -1) - (gamepad1.left_stick_x * xOffSet);
        double right = (gamepad1.left_stick_y * -1) + (gamepad1.left_stick_x * xOffSet);

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
        }else if(gamepad1.y){
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

        if(gamepad1.right_stick_x > (deadZone + 0.1)){
            robot.claw.setPosition(robot.closed);
        }else if(gamepad1.right_stick_x < ((deadZone + 0.1) * -1)){
            robot.claw.setPosition(robot.open);
        }

        if(gamepad1.left_trigger > 0){
            //set Fly Wheel To Spin Up if Left Trigger Is Held
            robot.fWheelPower(robot.highGoal);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }else if(gamepad1.left_bumper){
            //sets the fly wheel to fire the power shot
            fire(0, home);//0 is the power shot
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
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        }else if(gamepad1.right_trigger > 0 && velocity <= 100.0){
            //sets the  servo to fire
            robot.launcher.setPosition(robot.fire);
            //rev color
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
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
        telemetry.addData("Gyro: ", "" + robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("LED: ", robot.pattern.toString());
        telemetry.update();//call the display telemetry
    }

    public void fire(int state, float home){
        if(state == 0){
            robot.setPower(0.20,0.20);
            while(robot.color.blue() < 15 && robot.color.green() < 15 && robot.color.red() < 15){
                sleep(10);
                if(gamepad1.b){
                    break;
                }
            }
            robot.setPower(0,0);
            goToPosition(-1.5,0.20);
            turnTo(home,0.15);
            shoot(robot.powerShot - 90);
            turnTo(4,0.15);
            shoot(robot.powerShot - 100);
            turnTo(10,0.15);
            shoot(robot.powerShot - 120);
            robot.fWheelPower(0);
        }else{
            robot.setPower(0.20,0.20);
            while(robot.color.blue() < 15 && robot.color.green() < 15 && robot.color.red() < 15){
                sleep(10);
                if(gamepad1.b){
                    break;
                }
            }
            robot.setPower(0,0);
            goToPosition(-1.5,0.20);
            turnTo(home,0.15);
            shoot(robot.highGoal);
            shoot(robot.highGoal);
            shoot(robot.highGoal);
        }
    }

    public  void shoot(double power){
        robot.fWheelPower(power);
        double velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity
        for(int i = 0; i < 2; i++){
            while(velocity < (power - 10)){
                velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity update
                sleep(100);
                if(gamepad1.b){
                    break;
                }
            }
            while(velocity > (power + 10)){
                velocity = ((robot.fWheelOne.getVelocity() + robot.fWheelTwo.getVelocity()) / 2); //flywheels avg velocity update
                sleep(100);
                if(gamepad1.b){
                    break;
                }
            }
        }
        robot.launcher.setPosition(robot.fire);
        sleep(1000);
        robot.launcher.setPosition(robot.rest);
        sleep(250);
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
                    if(gamepad1.b){
                        break;
                    }
                }
            }else if(point < robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle){
                while ((point + 1) < robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) {
                    robot.setPower(power, power * -1);
                    sleep(10);
                    if(gamepad1.b){
                        break;
                    }
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
            if(gamepad1.b){
                break;
            }
        }
        robot.setPower(0,0);
        telemetry.addData("Running To Position", "Done");
        telemetry.update();
        robot.setMode(2);
    }
}

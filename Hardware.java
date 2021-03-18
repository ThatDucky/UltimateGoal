package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;

public class Hardware {
    //Revs Per Decimeters - variables
    double diameter = 10; //measure the wheel in centimeters
    double ticks = 28; //REV-41-129 has 28 ticks per cycle
    double ticksPerCentimeters = (ticks / (diameter * Math.PI)) * 12; //12 is for the gear reduction

    //flywheel variables
    double highGoal = 1900;
    double powerShot = 1000;

    //Sensors
    public ColorSensor color = null;
    public DistanceSensor dis = null;
    public BNO055IMU imu = null;

    public RevBlinkinLedDriver revBlinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

    //naming robot Drive motor
    public DcMotorEx one = null;
    public DcMotorEx two = null;
    public DcMotorEx three = null;
    public DcMotorEx four = null;

    public DcMotorEx fWheelOne = null; //left
    public DcMotorEx fWheelTwo = null; //right

    public DcMotor lift = null;
    public DcMotorEx arm = null;

    //naming servos
    public Servo launcher = null;
    public Servo claw = null;
    public Servo shove = null;

    //servo vars
    double rest = -0.15;
    double fire = 0.45;

    double open = 0.0;
    double closed = 0.70;

    double shoved = 0.95;
    double lay = 0.25;

    //constructor
    public Hardware(){

    }

    //initialize standard hardware interface
    public void init(HardwareMap hwMap){
        //imu sensor init
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        //Rev Blink init
        revBlinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "led");
        pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
        revBlinkinLedDriver.setPattern(pattern);

        //color sensor init
        color = hwMap.get(ColorSensor.class, "color");

        //distance init
        dis = hwMap.get(DistanceSensor.class,"dis"); //Ring Thickness is 2cm

        // define and initialize drive motors
        one = hwMap.get(DcMotorEx.class, "one");
        two = hwMap.get(DcMotorEx.class, "two");
        three = hwMap.get(DcMotorEx.class, "three");
        four = hwMap.get(DcMotorEx.class, "four");
        fWheelOne = hwMap.get(DcMotorEx.class, "fWheelOne");
        fWheelTwo = hwMap.get(DcMotorEx.class, "fWheelTwo");
        lift = hwMap.get(DcMotor.class, "autumn");
        arm = hwMap.get(DcMotorEx.class, "fall");

        //set motor mode to reset encoders
        setMode(0);
        fWheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fWheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set direction of the motors
        one.setDirection(DcMotorEx.Direction.FORWARD); //left Front
        two.setDirection(DcMotorEx.Direction.FORWARD); //left Back
        three.setDirection(DcMotorEx.Direction.REVERSE); //right Front
        four.setDirection(DcMotorEx.Direction.REVERSE); //right Back

        lift.setDirection(DcMotorEx.Direction.FORWARD); //ramp
        arm.setDirection(DcMotor.Direction.FORWARD); //arm

        fWheelOne.setDirection(DcMotorEx.Direction.FORWARD); //right flywheel
        fWheelTwo.setDirection(DcMotorEx.Direction.REVERSE); //left flywheel

        //set all drive motors to zero power
        setPower(0,0);
        fWheelPower(0);
        lift.setPower(0);
        arm.setPower(0);

        //define the Servos
        launcher = hwMap.get(Servo.class, "launcher");
        claw = hwMap.get(Servo.class, "claw");
        shove = hwMap.get(Servo.class, "shove");
    }

    public void fWheelPower(double power){
        //sets fly wheels power
        fWheelOne.setVelocity(power);
        fWheelTwo.setVelocity(power);
    }

    public void setPower(double lPower,double rPower){
        //set the power of all motors at once
        one.setPower(lPower);
        two.setPower(lPower);
        three.setPower(rPower);
        four.setPower(rPower);
    }

    public void setTargetPosition(double decimeters){
        //set the target position of all the motors at once
        int target = (int)(Math.round((decimeters * 10) * ticksPerCentimeters));
        one.setTargetPosition(one.getCurrentPosition() + target);
        two.setTargetPosition(two.getCurrentPosition() + target);
        three.setTargetPosition(three.getCurrentPosition() + target);
        four.setTargetPosition(four.getCurrentPosition() + target);
    }

    public String getTargetPosition(){
        //returns a string of all the target positions of the motors
        return one.getTargetPosition() + " " + two.getTargetPosition() + " " + three.getTargetPosition() + " " + four.getTargetPosition();
    }

    public boolean atTarget(){
        //returns true if any motor is at target postition
        if(one.getCurrentPosition() == one.getTargetPosition() || two.getCurrentPosition() == two.getTargetPosition() || three.getCurrentPosition() == three.getTargetPosition() || four.getCurrentPosition() == four.getTargetPosition()){
            return true;
        }
        return false;
    }

    public boolean isBusy(){
        //checks to see if any of the motor are in use and returns a bool
        return one.isBusy() || two.isBusy() || three.isBusy() || four.isBusy();
    }

    public void setMode(int mode){
        //sets the mode of all drive motors based on input
        switch (mode){
            case 0:
                //reset all encoders
                one.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                two.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                three.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                four.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case 1:
                //set to run to position mode
                one.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                two.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                three.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                four.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                break;
            case 2:
                //drive mode - run with encoders
                one.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                two.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                three.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                four.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                break;
            case 3:
                //drive mode - shut off encoders
                one.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                two.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                three.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                four.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }
    }

    public double calculateVelocity(double dx, double dy){
        double g = 9.8009; //Gravity
        double A = ((2.0*(Math.pow(dy,3.0))) + (9.0*g*(Math.pow(dx,2.0)))) / 108.0; //Section A
        double B = (-3.0 - (Math.pow(dy,2.0))) / 9.0; //Section B
        double velocity = Math.sin(Math.cbrt(A + Math.cbrt(Math.pow(A,2.0) + Math.pow(B,3.0))) + Math.cbrt(A - Math.sqrt(Math.pow(A,2.0) + Math.pow(B,3.0))) + (dy/3));
        velocity = ((100 * velocity) / (9 * Math.PI)) / ticks;
        return velocity;
    }

}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.lang.Math;

public class Hardware {
    //Revs Per Decimeters - variables
    double diameter = 10; //measure the wheel in centimeters
    double ticks = 28; //REV-41-129 has 28 ticks per cycle
    double ticksPerCentimeters = (ticks / (diameter * 3.1415)) * 12; //12 is for the gear reduction

    //flywheel variables
    double highGoal = 2200;
    double powerShot = 2060;

    //Sensors
    public ColorSensor color = null;
    public BNO055IMU imu = null;

    //naming robot Drive motor
    public DcMotorEx one = null;
    public DcMotorEx two = null;
    public DcMotorEx three = null;
    public DcMotorEx four = null;

    public DcMotorEx fWheelOne = null; //right
    public DcMotorEx fWheelTwo = null; //left

    //naming servos
    public Servo launcher = null;

    //servo vars
    public double rest = 0.0;
    public double fire = 0.45;

    //constructor
    public Hardware(){

    }

    //initialize standard hardware interface
    public void init(HardwareMap hwMap){
        //sensor setup
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        color = hwMap.get(ColorSensor.class, "color");

        // define and initialize drive motors
        one = hwMap.get(DcMotorEx.class, "one");
        two = hwMap.get(DcMotorEx.class, "two");
        three = hwMap.get(DcMotorEx.class, "three");
        four = hwMap.get(DcMotorEx.class, "four");
        fWheelOne = hwMap.get(DcMotorEx.class, "fWheelOne");
        fWheelTwo = hwMap.get(DcMotorEx.class, "fWheelTwo");

        //set motor mode to reset encoders
        setMode(0);
        fWheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fWheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set direction of the motors
        one.setDirection(DcMotorEx.Direction.FORWARD); //left Front
        two.setDirection(DcMotorEx.Direction.FORWARD); //left Back
        three.setDirection(DcMotorEx.Direction.REVERSE); //right Front
        four.setDirection(DcMotorEx.Direction.REVERSE); //right Back

        fWheelOne.setDirection(DcMotorEx.Direction.FORWARD); //right flywheel
        fWheelTwo.setDirection(DcMotorEx.Direction.REVERSE); //left flywheel

        //set all drive motors to zero power
        setPower(0,0);
        fWheelPower(0);

        //define the Servos
        launcher = hwMap.get(Servo.class, "launcher");
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
}
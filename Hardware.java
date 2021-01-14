package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

public class Hardware {
    //Revs Per Decimeters - variables
    double diameter = 10; //measure the wheel in centimeters
    double ticks = 28; //REV-41-129 has 28 ticks per cycle
    double ticksPerCentimeters = ticks / (diameter * 3.1415); //1.12 cm per tick or 0.89 ticks per cm

    //flywheel variables
    double highGoal = 0.92;
    double powerShot = 0.85;

    //naming robot Drive motor
    public DcMotorEx one = null;
    public DcMotorEx two = null;

    public DcMotorEx fWheelOne = null; //right
    public DcMotorEx fWheelTwo = null; //left

    //naming servos
    public Servo launcher = null;

    //servo vars
    public double rest = 0.0;
    public double fire = 0.45;

    //local op mode members
    HardwareMap hwMap = null;

    //constructor
    public Hardware(){

    }

    //initialize standard hardware interface
    public void init(HardwareMap hwMap){
        // define and initialize drive motors
        one = hwMap.get(DcMotorEx.class, "one");
        two = hwMap.get(DcMotorEx.class, "two");
        fWheelOne = hwMap.get(DcMotorEx.class, "fWheelOne");
        fWheelTwo = hwMap.get(DcMotorEx.class, "fWheelTwo");

        //set motor mode to reset encoders
        setMode(0);
        fWheelOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fWheelTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set direction of the motors
        one.setDirection(DcMotorEx.Direction.FORWARD); //left Front
        two.setDirection(DcMotorEx.Direction.REVERSE); //left Back
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
        fWheelOne.setPower(power);
        fWheelTwo.setPower(power);
    }

    public void setPower(double lPower,double rPower){
        //set the power of all motors at once
        one.setPower(lPower);
        two.setPower(rPower);
    }

    public void setTargetPosition(double decimeters){
        //set the target position of all the motors at once
        int target = (int)(Math.round((decimeters * 10) * ticksPerCentimeters));
        one.setTargetPosition(one.getCurrentPosition() + target);
        two.setTargetPosition(two.getCurrentPosition() + target);
    }
    public String getTargetPosition(){
        //returns a string of all the target positions of the motors
        return one.getTargetPosition() + " " + two.getTargetPosition();
    }

    public boolean isBusy(){
        //checks to see if any of the motor are in use and returns a bool
        return one.isBusy() || two.isBusy();
    }

    public void setMode(int mode){
        //sets the mode of all drive motors based on input
        switch (mode){
            case 0:
                //reset all encoders
                one.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                two.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case 1:
                //set to run to position mode
                one.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                two.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                break;
            case 2:
                //drive mode - run with encoders
                one.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                two.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                break;
            case 3:
                //drive mode - shut off encoders
                one.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                two.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }
    }
}
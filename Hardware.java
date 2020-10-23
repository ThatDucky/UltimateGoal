package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

public class Hardware{
    //Revs Per Decimeters - variables
    double diameter = 1; //measure the wheel in decimeter
    double ticks = 28; //REV-41-129 has 28 ticks per cycle
    double ticksPerDecimeters = ticks / (diameter * 3.1415);

    //naming robot Drive motor
    public DcMotorEx one = null;
    public DcMotorEx two = null;
    public DcMotorEx three = null;
    public DcMotorEx four = null;

    //naming servos
    //public Servo  = null;

    //local op mode members
    HardwareMap hwMap = null;

    //constructor
    public Hardware(){

    }

    //initialize standard hardware interface
    public void init(HardwareMap ahwMap){
        //save reference to hardware map
        hwMap = ahwMap;

        // define and initialize drive motors
        one = hwMap.get(DcMotorEx.class, "one");
        two = hwMap.get(DcMotorEx.class, "two");
        three = hwMap.get(DcMotorEx.class, "three");
        four = hwMap.get(DcMotorEx.class, "four");

        //set motor mode to reset encoders
        setMode(0);

        //set direction of the motors
        one.setDirection(DcMotorEx.Direction.FORWARD); //left Front
        two.setDirection(DcMotorEx.Direction.FORWARD); //left Back
        three.setDirection(DcMotorEx.Direction.FORWARD); //right Front
        four.setDirection(DcMotorEx.Direction.FORWARD); //right Back

        //set all drive motors to zero power
        setPower(0,0);

        //define the Servos
        // = hwMap.get(Servo.class, "");
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
        int target = (int)(decimeters * ticksPerDecimeters);
        one.setTargetPosition(one.getCurrentPosition() + target);
        two.setTargetPosition(two.getCurrentPosition() + target);
        three.setTargetPosition(three.getCurrentPosition() + target);
        four.setTargetPosition(four.getCurrentPosition() + target);
    }

    public void setMode(int mode){
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
                //drive mode - shut off encoders
                one.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                two.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                three.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                four.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }
    }
}
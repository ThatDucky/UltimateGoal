package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware{
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

        //set motor mode
        one.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        two.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        three.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        four.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //set direction of the motors
        one.setDirection(DcMotorEx.Direction.FORWARD); //left Front
        two.setDirection(DcMotorEx.Direction.FORWARD); //left Back
        three.setDirection(DcMotorEx.Direction.REVERSE); //right Front
        four.setDirection(DcMotorEx.Direction.REVERSE); //right Back

        //set all drive motors to zero power
        setPower(0);

        //define the Servos
        // = hwMap.get(Servo.class, "");
    }

    public void setPower(double power){
        //set the power of all motors at once
        one.setPower(power);
        two.setPower(power);
        three.setPower(power);
        four.setPower(power);
    }

    public void setTargetPosition(int target){
        //set the target postion of all the motors at once
        one.setTargetPosition(one.getCurrentPosition() + target);
        two.setTargetPosition(two.getCurrentPosition() + target);
        three.setTargetPosition(three.getCurrentPosition() + target);
        four.setTargetPosition(four.getCurrentPosition() + target);
    }

    public void reset(){
        //reset all the encoders
        one.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        two.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        three.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        four.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

}
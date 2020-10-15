package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    //naming robot Drive motor
    public DcMotor one = null;

    //naming servos
    //public Servo  = null;

    //servo vars

    //local op mode members
    HardwareMap hwMap = null;

    //constructor
    public Hardware () {
    }

    //initialize standard hardware interface
    public void init(HardwareMap ahwMap){

        //save reference to hardware map
        hwMap = ahwMap;

        // define and initialize drive motors
        one = hwMap.get(DcMotor.class, "one");

        //set direction of the motors
        one.setDirection(DcMotor.Direction.REVERSE);

        //set all drive motors to zero power
        one.setPower(0);

        //define the Servos
        //leftArm = hwMap.get(Servo.class, "");
    }


}
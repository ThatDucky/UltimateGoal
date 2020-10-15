import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    //naming robot Drive motor
    public DcMotor  = null;

    //naming servos
    public Servo  = null;

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
        driveOne = hwMap.get(DcMotor.class, "");

        //set direction of the motors
        driveOne.setDirection(DcMotor.Direction.REVERSE);

        //set all drive motors to zero power
        driveOne.setPower(0);

        //define the Servos
        leftArm = hwMap.get(Servo.class, "");
    }


}
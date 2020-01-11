package TeamCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSys_Elephant {
    DcMotor eTrunk;
    Servo eNose;
    Servo eTail;
    private double eTrunkScaler = 1;
    public int maxTrunkHeight = 6100;
    HardwareMap hardwareMap;

    SubSys_Elephant(){}

    public void init(HardwareMap hM){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        this.hardwareMap = hM;
        eTrunk = hardwareMap.get(DcMotor.class, "eTrunk");
        eTrunk.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eTrunk.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eTrunk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               eNose = hardwareMap.get(Servo.class, "eNose");
        eTail = hardwareMap.get(Servo.class, "eTail");
        eNose.setPosition(0);
        eTail.setPosition(0);


    }


    public void moveTrunk(Gamepad gamepad){
        eTrunkScaler = 1-0.5*eTrunk.getCurrentPosition()/maxTrunkHeight;
        double output;
        //Left bumper is for raising
        //Right bumper is for lowering
        if (eTrunk.getCurrentPosition() >0 && gamepad.left_trigger > 0 && gamepad.right_trigger == 0)
            output = -gamepad.left_trigger* eTrunkScaler;  //should only move forward if limit switch not pressed and only right trigger is
        else if(eTrunk.getCurrentPosition()<maxTrunkHeight && gamepad.right_trigger > 0 && gamepad.left_trigger == 0) // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            output = gamepad.right_trigger* eTrunkScaler;    //should only move forward if limit switch not pressed and only left trigger is
        else
            output = 0;
        eTrunk.setPower(ramp_Motor_Power(eTrunk.getPower(), output));
    }


    public void moveNose(Gamepad gamepad){
        double servoPosL = eNose.getPosition();
        if(gamepad.left_bumper == true){ //to open mouth
            eNose.setPosition(servoPosL-0.05);
        } else if(gamepad.right_bumper == true){ //to close mouth
            eNose.setPosition(servoPosL+0.05);
        }
    }

    public void moveTail(Gamepad gamepad){
        double servoPos = eTail.getPosition();
        //double servoPos = 0;
        if(gamepad.a == true){ //to lower tail
            eTail.setPosition(servoPos+0.01);
        } else if(gamepad.b == true){ //to raise tail
            eTail.setPosition(servoPos-0.01);
        }

    }

    private double ramp_Motor_Power(double current_Power, double desired_Power){
        double diff = desired_Power-current_Power;
        if(Math.abs(desired_Power) <0.2 || Math.abs(diff)<0.05)
            current_Power = desired_Power;
        else
            current_Power += (diff/(Math.abs(diff)))*0.05;
        return current_Power;
    }

}

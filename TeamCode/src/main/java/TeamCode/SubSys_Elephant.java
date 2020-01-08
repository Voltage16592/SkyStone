package TeamCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSys_Elephant {
    DcMotor eTrunk;
    DigitalChannel forwardLimitSwitch;
    DigitalChannel reverseLimitSwitch;
    Servo eNoseL;
    Servo eNoseR;
    Servo eTail;
    private double eTrunkScaler = 0.4;
    HardwareMap hardwareMap;

    SubSys_Elephant(){}

    public void init(HardwareMap hM){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        this.hardwareMap = hM;
        forwardLimitSwitch = hardwareMap.get(DigitalChannel.class, "forwardLimitSwitch");
        reverseLimitSwitch = hardwareMap.get(DigitalChannel.class, "reverseLimitSwitch");
        eTrunk = hardwareMap.get(DcMotor.class, "eTrunk");
        eTrunk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eTrunk.setDirection(DcMotorSimple.Direction.REVERSE);
        eNoseL = hardwareMap.get(Servo.class, "eNoseL");
        eNoseR = hardwareMap.get(Servo.class, "eNoseR");
        eTail = hardwareMap.get(Servo.class, "eTail");
        eNoseL.setPosition(0);
        eNoseR.setPosition(0);


    }

    public void moveTrunk(Gamepad gamepad){
        double output;
        //Left bumper is for raising
        //Right bumper is for lowering
        if (!isDetected(reverseLimitSwitch) && gamepad.left_trigger > 0 && gamepad.right_trigger == 0)
            output = gamepad.left_trigger* eTrunkScaler;  //should only move forward if limit switch not pressed and only right trigger is
        else if(!isDetected(forwardLimitSwitch) && gamepad.right_trigger > 0 && gamepad.left_trigger == 0) // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            output = -gamepad.right_trigger* eTrunkScaler;    //should only move forward if limit switch not pressed and only left trigger is
        else
            output = 0;
        eTrunk.setPower(ramp_Motor_Power(eTrunk.getPower(), output));
    }

    public void moveNose(Gamepad gamepad){
        double servoPosL = eNoseL.getPosition();
        double servoPosR = eNoseR.getPosition();
        if(gamepad.left_bumper == true){ //to open mouth
            eNoseL.setPosition(servoPosL+0.05);
            eNoseR.setPosition(servoPosR-0.05);
        } else if(gamepad.right_bumper == true){ //to close mouth
            eNoseL.setPosition(servoPosL-0.05);
            eNoseR.setPosition(servoPosR+0.05);
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

    public boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    } // for magnetic limit switches

    private double ramp_Motor_Power(double current_Power, double desired_Power){
        double diff = desired_Power-current_Power;
        if(Math.abs(desired_Power) <0.2 || Math.abs(diff)<0.05)
            current_Power = desired_Power;
        else
            current_Power += (diff/(Math.abs(diff)))*0.05;
        return current_Power;
    }

}

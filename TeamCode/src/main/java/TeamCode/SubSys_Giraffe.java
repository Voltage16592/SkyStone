package TeamCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSys_Giraffe {
    DcMotor gNeck;
    DigitalChannel forwardLimitSwitch;
    DigitalChannel reverseLimitSwitch;
    Servo giraffeMouth;
    Servo giraffeTail;
    private double giraffeScaler = 0.3;
    HardwareMap hardwareMap;

    SubSys_Giraffe(){}

    public void init(HardwareMap hM){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        this.hardwareMap = hM;
        forwardLimitSwitch = hardwareMap.get(DigitalChannel.class, "forwardLimitSwitch");
        reverseLimitSwitch = hardwareMap.get(DigitalChannel.class, "reverseLimitSwitch");
        gNeck = hardwareMap.get(DcMotor.class, "gNeck");
        giraffeMouth = hardwareMap.get(Servo.class, "giraffeMouth");
        giraffeTail = hardwareMap.get(Servo.class, "giraffeTail");


    }

    public void limit(Gamepad gamepad1){
        double output;
        //Left bumper is for raising
        //Right bumper is for lowering
        if (!isDetected(reverseLimitSwitch) && gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0)
            output = gamepad1.left_trigger*giraffeScaler;  //should only move forward if limit switch not pressed and only right trigger is
        else if(!isDetected(forwardLimitSwitch) && gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            output = -gamepad1.right_trigger*giraffeScaler;    //should only move forward if limit switch not pressed and only left trigger is
        else
            output = 0;
        gNeck.setPower(ramp_Motor_Power(gNeck.getPower(), output));
    }

    public void moveMouth(Gamepad gamepad1){
        double servoPos = giraffeMouth.getPosition();
        if(gamepad1.left_bumper == true){ //to close mouth
            giraffeMouth.setPosition(servoPos-0.05);
        } else if(gamepad1.right_bumper == true){ //to open mouth
            giraffeMouth.setPosition(servoPos+0.05);
        }
    }

    public void moveTail(Gamepad gamepad1){
        double servoPos = giraffeTail.getPosition();
        //double servoPos = 0;
        if(gamepad1.a == true && giraffeTail.getPosition()>0.1){ //to lower tail
            giraffeTail.setPosition(servoPos-0.01);
        } else if(gamepad1.b == true){ //to raise tail
            giraffeTail.setPosition(servoPos+0.01);
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

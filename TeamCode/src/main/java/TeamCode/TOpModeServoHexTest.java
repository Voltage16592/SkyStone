package TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

@TeleOp(name="TOpModeServoHexTest", group="Iterative Opmode")
@Disabled
public class TOpModeServoHexTest extends OpMode {
    private DcMotor georgeIsBad = null;
    private Servo servo1 = null;

    public void init(){
        georgeIsBad = hardwareMap.get(DcMotor.class, "georgeIsBad");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        georgeIsBad.setDirection(FORWARD);
        servo1.setDirection(Servo.Direction.FORWARD);
        servo1.setPosition(0);
        telemetry.addData("Status", "Running");
    }
    public void loop(){
        double MotorPower = -gamepad1.left_stick_y*0.5;

        georgeIsBad.setPower(MotorPower);
        double servoPos = servo1.getPosition();
        if(gamepad1.dpad_down == true){
            servo1.setPosition(servoPos-0.01);
        }
        if(gamepad1.dpad_up == true){
            servo1.setPosition(servoPos+0.01);
        }

    }

    @Override
    public void stop() {
        georgeIsBad.setPower(0);


    }
}

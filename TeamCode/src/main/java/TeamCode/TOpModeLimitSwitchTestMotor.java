//Created by George WAng
// /* Copyright (c) 2017 FIRST. All rights reserved.

package TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TOpModeLimitSwitchTestMotor", group="Iterative Opmode")

//when disabled won't exist
@Disabled
public class TOpModeLimitSwitchTestMotor extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm;
    private DigitalChannel forwardLimitSwitch;
    private DigitalChannel reverseLimitSwitch;
    private double armScaler = 0.4;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        forwardLimitSwitch = hardwareMap.get(DigitalChannel.class, "forwardLimitSwitch");
        reverseLimitSwitch = hardwareMap.get(DigitalChannel.class, "reverseLimitSwitch");
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Running");
        telemetry.update();
    }


    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        //double output = this.gamepad1.right_stick_y(); //Moves the joystick based on Y value
        double output = -this.gamepad1.right_stick_y*armScaler;
        if (isDetected(forwardLimitSwitch)) // If the forward limit switch is pressed, we want to keep the values between -1 and 0
            output = 0;
        else if(isDetected(reverseLimitSwitch)) // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            output = 0;
        arm.setPower(output);
        report();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    }

    private void report(){
        if(isDetected(forwardLimitSwitch)){
            telemetry.addData("forwardLimitSwitch", "detected");
        } else{
            telemetry.addData("forwardLimitSwitch", "not detected");
        }
        if(isDetected(reverseLimitSwitch)){
            telemetry.addData("reverseLimitSwitch","detected");
        } else{
            telemetry.addData("reverseLimitSwitch", "not detected");

        }
        telemetry.update();
    }
}

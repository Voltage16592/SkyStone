/* Created by Lucas Wu and Mira Chew
 * Mode which includes chassis, arm, and claw movement
 */

//version 1

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TOpMode_FullBot", group="Iterative Opmode")
//@Disabled
public class TOpMode_FullBot
        extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor gNeck;
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DigitalChannel forwardLimitSwitch;
    private DigitalChannel reverseLimitSwitch;
    private Servo giraffeMouth;
    private double giraffeScaler = 0.4;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        forwardLimitSwitch = hardwareMap.get(DigitalChannel.class, "forwardLimitSwitch");
        reverseLimitSwitch = hardwareMap.get(DigitalChannel.class, "reverseLimitSwitch");
        gNeck = hardwareMap.get(DcMotor.class, "gNeck");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        gNeck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Running");
        giraffeMouth = hardwareMap.get(Servo.class, "giraffeMouth");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
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

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        limit();
        simpleTankDrive();
        giraffeMouthMovement();
        report();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void limit(){
        double output;
        //Left bumper is for raising
        //Right bumper is for lowering
        if (!isDetected(reverseLimitSwitch) && this.gamepad1.left_trigger > 0 && this.gamepad1.right_trigger == 0)
            output = this.gamepad1.left_trigger*giraffeScaler;  //should only move forward if limit switch not pressed and only right trigger is
        else if(!isDetected(forwardLimitSwitch) && this.gamepad1.right_trigger > 0 && this.gamepad1.left_trigger == 0) // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            output = -this.gamepad1.right_trigger*giraffeScaler;    //should only move forward if limit switch not pressed and only left trigger is
        else
            output = 0;
        gNeck.setPower(output);
    }

    private void report() {
        if (isDetected(forwardLimitSwitch)) {
            telemetry.addData("forwardLimitSwitch", "detected");
        } else {
            telemetry.addData("forwardLimitSwitch", "not detected");
        }
        if (isDetected(reverseLimitSwitch)) {
            telemetry.addData("reverseLimitSwitch", "detected");
        } else {
            telemetry.addData("reverseLimitSwitch", "not detected");

        }
        telemetry.addData("Left Stick Value:", -gamepad1.left_stick_y);
        telemetry.addData("Right Stick Value:", -gamepad1.right_stick_y);

        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Right Bumper", gamepad1.right_bumper);

        telemetry.addData("Servo Position:", giraffeMouth.getPosition());
        telemetry.update();
    }

    private void simpleTankDrive(){
        right_drive.setPower(-gamepad1.right_stick_y);
        left_drive.setPower(gamepad1.left_stick_y);

    }

    private void giraffeMouthMovement(){
        double servoPos = giraffeMouth.getPosition();
        if(gamepad1.left_bumper == true){ //to close mouth
            giraffeMouth.setPosition(servoPos-0.01);
        } else if(gamepad1.right_bumper == true){ //to open mouth
            giraffeMouth.setPosition(servoPos+0.01);
        }

    }

    private boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    } // for magnetic limit switches
}

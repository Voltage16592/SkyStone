/*Created by Lucas Wu (including stuff stolen from George and Mira)
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autjonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TOpModeTankDriveClawAndArmWLimitSwitches", group="Iterative Opmode")
//@Disabled


public class TOpModeTankDriveClawAndArmWLimitSwitches extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor georgeIsBad = null;
    private Servo servo1 = null;
    private SensorDigitalTouch limitSwitchForward = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Motor - Tank Drive

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        limitSwitchForward = hardwareMap.get(SensorDigitalTouch.class, "limitSwitchForward");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        //Claw and Arm Setup
        georgeIsBad = hardwareMap.get(DcMotor.class, "georgeIsBad");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        georgeIsBad.setDirection(FORWARD);
        georgeIsBad.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo1.setDirection(Servo.Direction.FORWARD);
        servo1.setPosition(0);
        telemetry.addData("Status", "Running");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        //Drive

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = -gamepad1.left_stick_y*0.5;
        double rightPower = -gamepad1.right_stick_y*0.5;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        telemetry.addData("left stick y", gamepad1.left_stick_y);
        telemetry.addData("right stick y", gamepad1.right_stick_y);
        telemetry.update();

        // Arm and Claw

        // Need to add limit switches before using the arm
        // double MotorPower = -gamepad1.left_stick_y*0.5;
        // georgeIsBad.setPower(MotorPower);

        double servoPos = servo1.getPosition();
        if(gamepad1.dpad_down == true){
            servo1.setPosition(servoPos-0.01);
        }
        if(gamepad1.dpad_up == true){
            servo1.setPosition(servoPos+0.01);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }

}

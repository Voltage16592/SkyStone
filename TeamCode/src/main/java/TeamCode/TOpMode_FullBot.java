/* Created by Lucas Wu and Mira Chew
 * Mode which includes chassis, arm, and claw movement
 */

//version 1

package TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TOpMode_FullBot", group="Iterative Opmode")
//@Disabled
public class TOpMode_FullBot
        extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor gNeck;
    private DigitalChannel forwardLimitSwitch;
    private DigitalChannel reverseLimitSwitch;
    private Servo giraffeMouth;
    private Servo giraffeTail;
    private double giraffeScaler = 0.3;
    private SubSys_TankDrive tankDrive = new SubSys_TankDrive();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        tankDrive.init(hardwareMap);
        forwardLimitSwitch = hardwareMap.get(DigitalChannel.class, "forwardLimitSwitch");
        reverseLimitSwitch = hardwareMap.get(DigitalChannel.class, "reverseLimitSwitch");
        gNeck = hardwareMap.get(DcMotor.class, "gNeck");
        gNeck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Running");
        giraffeMouth = hardwareMap.get(Servo.class, "giraffeMouth");
        giraffeTail = hardwareMap.get(Servo.class, "giraffeTail");

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
        tankDrive.move(gamepad1.left_stick_y, -gamepad1.right_stick_y);
        giraffeMouthMovement();
        giraffeTailMovement();
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
        if (!isDetected(reverseLimitSwitch) && gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0)
            output = gamepad1.left_trigger*giraffeScaler;  //should only move forward if limit switch not pressed and only right trigger is
        else if(!isDetected(forwardLimitSwitch) && gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            output = -gamepad1.right_trigger*giraffeScaler;    //should only move forward if limit switch not pressed and only left trigger is
        else
            output = 0;
        gNeck.setPower(ramp_Motor_Power(gNeck.getPower(), output));
    }

    private void report() {
        if (isDetected(forwardLimitSwitch)) {
            //telemetry.addData("forwardLimitSwitch", "detected");
        } else {
            //telemetry.addData("forwardLimitSwitch", "not detected");
        }
        if (isDetected(reverseLimitSwitch)) {
            //telemetry.addData("reverseLimitSwitch", "detected");
        } else {
            //telemetry.addData("reverseLimitSwitch", "not detected");

        }
        telemetry.addData("left stick reading", gamepad1.left_stick_y);
        telemetry.addData("left_drive power", tankDrive.getLeft_Drive().getPower());
        //telemetry.addData("Left S  mm stick Value:", -gamepad1.left_stick_y);
        //telemetry.addData("Right Stick Value:", -gamepad1.right_stick_y);

        //telemetry.addData("Left Bumper", gamepad1.left_bumper);
        //jhtelemetry.addData("Right Bumper", gamepad1.right_bumper);

        telemetry.addData("giraffeTail Position:", giraffeTail.getPosition());
        telemetry.update();
    }

    /*
    private void simpleTankDrive(){
        //if both sticks are within a certain reading (close enough) should make equalize powers to larger magnitude
        double right_Desired_Power = -gamepad1.right_stick_y;
        double left_Desired_Power = gamepad1.left_stick_y;
        if(Math.abs(right_Desired_Power-left_Desired_Power)<=0.08){
            if(Math.abs(right_Desired_Power)>Math.abs(left_Desired_Power))
                left_Desired_Power = right_Desired_Power;
            else
                right_Desired_Power = left_Desired_Power;
        }

        //Only want to ramp power if increasing speed
        if(Math.abs(right_Desired_Power) > Math.abs(right_drive.getPower()) && Math.abs(left_Desired_Power) > Math.abs(left_drive.getPower())) {
            right_drive.setPower(ramp_Motor_Power(right_drive.getPower(), right_Desired_Power));
            left_drive.setPower(ramp_Motor_Power(left_drive.getPower(), left_Desired_Power));
        } else  {
            right_drive.setPower(-gamepad1.right_stick_y);
            left_drive.setPower(gamepad1.left_stick_y);
        }

    }
    */

    private void giraffeMouthMovement(){
        double servoPos = giraffeMouth.getPosition();
        if(gamepad1.left_bumper == true){ //to close mouth
            giraffeMouth.setPosition(servoPos-0.05);
        } else if(gamepad1.right_bumper == true){ //to open mouth
            giraffeMouth.setPosition(servoPos+0.05);
        }

    }

    private void giraffeTailMovement(){
        double servoPos = giraffeTail.getPosition();
        //double servoPos = 0;
        if(gamepad1.a == true && giraffeTail.getPosition()>0.1){ //to lower tail
            giraffeTail.setPosition(servoPos-0.01);
        } else if(gamepad1.b == true){ //to raise tail
            giraffeTail.setPosition(servoPos+0.01);
        }

    }

    private boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    } // for magnetic limit switches

    private double ramp_Motor_Power(double current_Power, double desired_Power){
        double diff = desired_Power-current_Power;
        if (diff > 0.04)
            current_Power += 0.04;
        else if (diff < -0.04)
            current_Power -= 0.04;
        else
            current_Power = desired_Power;
        return current_Power;
    }//to ramp power instead of going 0 to 100
}

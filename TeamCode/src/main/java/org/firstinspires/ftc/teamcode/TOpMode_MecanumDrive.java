/* Created by Lucas Wu and Mira Chew
 * Mode which includes chassis, arm, and claw movement
 */

//version 1

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TOpMode_FullBot", group="Iterative Opmode")
//@Disabled
public class TOpMode_MecanumDrive
        extends OpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fleft_drive;
    private DcMotor fright_drive;
    private DcMotor bleft_drive;
    private DcMotor bright_drive;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Running");


        fleft_drive = hardwareMap.get(DcMotor.class, "fleft_drive");
        fright_drive = hardwareMap.get(DcMotor.class, "fright_drive");
        bleft_drive = hardwareMap.get(DcMotor.class, "bleft_drive");
        bright_drive = hardwareMap.get(DcMotor.class, "bright_drive");

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

        simpleMecDrive();

        report();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    private void report() {

        telemetry.addData("right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);


        telemetry.update();
    }

    private void simpleMecDrive(){

        double fwd_bkwd;
        double rt_lt;
        //boolean right_bumper;
        //boolean left_bumper;
        fwd_bkwd = -gamepad1.right_stick_y;
        rt_lt = -gamepad1.right_stick_x; //should not be negative but already coded everything to compensate so x axis is "reversed"
        //right_bumper =  rt.gamepad1().right_bumper();
        //left_bumper =  rt.gamepad1().left_bumper();


        if ((fwd_bkwd <-0.25 || fwd_bkwd>0.25) &&rt_lt >-0.25 && rt_lt <0.25 ) {
            setMotorPowerAll(fwd_bkwd,fwd_bkwd, fwd_bkwd, fwd_bkwd);
        }   //Moves motor forward and backward

        if ((rt_lt <-0.25 || rt_lt >0.25) && fwd_bkwd >-0.25 && fwd_bkwd<0.25){
            setMotorPowerAll(-rt_lt,rt_lt, rt_lt, -rt_lt);
        }   //Moves robot side to side (strafe)

        if ((fwd_bkwd > 0.25 && rt_lt < -0.25) || (fwd_bkwd < -0.25 && rt_lt > 0.25)) {
            setMotorPowerAll((-rt_lt + fwd_bkwd)*2/3,0, 0, (-rt_lt + fwd_bkwd)*2/3);
        }   //this moves the robot at a 45 degree angle (hence the number 45 in telemetry)

        if ((fwd_bkwd > 0.25 && rt_lt > 0.25) || (fwd_bkwd < -0.25 && rt_lt < -0.25)) {
            setMotorPowerAll(0,(rt_lt + fwd_bkwd)*2/3,(rt_lt + fwd_bkwd)*2/3, 0 );

        }   //this moves the robt at a 135 degree angle (hence the number 135 in telemetry)

        if (fwd_bkwd<0.25 && fwd_bkwd>-0.25 && rt_lt<0.25 && rt_lt>-0.25 && !gamepad1.right_bumper && !gamepad1.left_bumper) {
            setMotorPowerAll(0,0, 0, 0);
        }   //When no buttons are pressed, all motors stop

        if (gamepad1.right_trigger>0) {         //right bumper makes the robot spin clockwise
            setMotorPowerAll(gamepad1.right_trigger,-gamepad1.right_trigger, gamepad1.right_trigger, -gamepad1.right_trigger);
        }else if (gamepad1.left_trigger>0) {    //left bumper makes the robot spin counterclockwise
            setMotorPowerAll(-gamepad1.left_trigger,gamepad1.left_trigger, -gamepad1.left_trigger, gamepad1.left_trigger);
        }



        //Only want to ramp power if increasing speed
        /*
        if(Math.abs(right_Desired_Power) > Math.abs(fright_drive.getPower()) && Math.abs(left_Desired_Power) > Math.abs(fleft_drive.getPower())) {
            fright_drive.setPower(ramp_Motor_Power(fright_drive.getPower(), right_Desired_Power));
            fleft_drive.setPower(ramp_Motor_Power(fleft_drive.getPower(), left_Desired_Power));
        } else  {
            fright_drive.setPower(-gamepad1.right_stick_y);
            fleft_drive.setPower(gamepad1.left_stick_y);
        }
        */

    }

    private void setMotorPowerAll(double fl, double fr, double bl, double br) {
        fleft_drive.setPower(fl);
        fright_drive.setPower(fr);
        bleft_drive.setPower(bl);
        bright_drive.setPower(br);
    }

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

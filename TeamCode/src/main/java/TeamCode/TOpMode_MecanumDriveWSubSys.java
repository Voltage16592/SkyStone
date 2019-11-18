/* Created by Lucas Wu and Mira Chew
 * Mode which includes chassis, arm, and claw movement
 */

//version 1

package TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TOpMode_MecanumDriveWSubSys", group="Iterative Opmode")
//@Disabled
public class TOpMode_MecanumDriveWSubSys
        extends OpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private SubSys_MecDrive mecDrive = new SubSys_MecDrive();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Running");


        mecDrive.init(hardwareMap);


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

        mecDrive.move(-gamepad1.right_stick_y, -gamepad1.right_stick_x, gamepad1.right_trigger, gamepad1.left_trigger);



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

/* Created by Lucas Wu and Mira Chew
 * Mode which includes chassis, arm, and claw movement
 */

//version 1

package TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TOpMode_ElephBot", group="Iterative Opmode")
//@Disabled
public class TOpMode_ElephBot
        extends OpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private SubSys_MecDrive mecDrive = new SubSys_MecDrive();
    private SubSys_Elephant elephant = new SubSys_Elephant();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Running");


        mecDrive.init(hardwareMap);
        elephant.init(hardwareMap);

        elephant.eTail.setPosition(0.5);

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


        if(!(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.right_bumper || gamepad1.left_bumper))
            mecDrive.joystickDrive(-gamepad1.right_stick_y,   -gamepad1.right_stick_x, gamepad1.right_trigger, gamepad1.left_trigger);
        mecDrive.precisionDrive(gamepad1.dpad_up, gamepad1.dpad_down,   gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.right_bumper, gamepad1.left_bumper);


        elephant.moveTrunk(gamepad2);
        elephant.moveNose(gamepad2);
        elephant.moveTail(gamepad2);

        report();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    private void report() {

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("trunk encoder pos", elephant.eTrunk.getCurrentPosition());
        telemetry.addData("right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("left_trigger", gamepad1.left_trigger);

        telemetry.addData("up", gamepad1.dpad_up);


        telemetry.update();
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

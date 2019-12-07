/* Created by Lucas Wu and Mira Chew
 * Mode which includes chassis, arm, and claw movement
 */

//version 1

package TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TOpMode_GiraffeBotWSubSys", group="Iterative Opmode")
@Disabled
public class TOpMode_GiraffeBotWSubSys
        extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private SubSys_TankDrive tankDrive = new SubSys_TankDrive();
    private SubSys_Giraffe giraffe = new SubSys_Giraffe();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Running");
        tankDrive.init(hardwareMap);
        giraffe.init(hardwareMap);
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
        giraffe.limit(gamepad1);
        tankDrive.move(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        giraffe.moveMouth(gamepad1);
        giraffe.moveTail(gamepad1);
        report();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }



    private void report() {
        if (giraffe.isDetected(giraffe.forwardLimitSwitch)) {
            //telemetry.addData("forwardLimitSwitch", "detected");
        } else {
            //telemetry.addData("forwardLimitSwitch", "not detected");
        }
        if (giraffe.isDetected(giraffe.reverseLimitSwitch)) {
            //telemetry.addData("reverseLimitSwitch", "detected");
        } else {
            //telemetry.addData("reverseLimitSwitch", "not detected");

        }
        telemetry.addData("left stick reading", gamepad1.left_stick_y);
        telemetry.addData("left_drive power", tankDrive.left_drive.getPower());
        telemetry.addData("right stick reading", gamepad1.right_stick_y);
        telemetry.addData("right_drive power", tankDrive.right_drive.getPower());


        telemetry.addData("eTail Position:", giraffe.giraffeTail.getPosition());
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
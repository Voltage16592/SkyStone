package TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="TOpModeSimpleLimitSwitchTest", group="Iterative Opmode")
public class TOpModeSimpleLimitSwitchTest extends LinearOpMode {

    private DigitalChannel forwardLimitSwitch;
    private DigitalChannel reverseLimitSwitch;


    @Override
    public void runOpMode() {

        forwardLimitSwitch = hardwareMap.get(DigitalChannel.class, "forwardLimitSwitch");
        reverseLimitSwitch = hardwareMap.get(DigitalChannel.class, "reverseLimitSwitch");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until player plays stop

        //return status of the limit switch
        while (opModeIsActive()) {
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

    //since magnetic limit switch is reverse, this method returns a reversed value
    private boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    }
}
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

@TeleOp
//@Disabled
public class TOpModeTelemetry extends LinearOpMode {


    private DigitalChannel digIn;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private SensorDigitalTouch limitSwitchForward = hardwareMap.get(SensorDigitalTouch.class, "limitSwitchForward");

    public void TOpModeTelemetry(String args[]){

        runOpMode();


    }


    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized Telemetry Mode");
        telemetry.update();

        // Set up limit switch
        try {
            telemetry = robot.telemetry;
            hardwareMap = robot.hwMap;
            digIn = hardwareMap.get(DigitalChannel.class, "digin");
            telemetry.addData("<", "Fully initialized");
        }
        catch(Exception e) {
            telemetry.addData("No Worko", "Can't make digIn");
        }

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("right stick y", gamepad1.right_stick_y);
            telemetry.addData("limitSwitchForward", limitSwitchForward);

            try {
                telemetry.addData("Is pressed?", digIn.getState());
            }
            catch(Exception e){
                telemetry.addData("No Worko", "Can't make digIn");
            }




        }
    }
}

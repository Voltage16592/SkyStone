package TeamCode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AOpMode_SimpleElephMove", group="Linear Opmode")
//@Disabled
public class AOpMode_SimpleElephMove extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();
    private SubSys_MecDrive mecDrive = new SubSys_MecDrive();





    @Override
    public void runOpMode() {


        mecDrive.init(hardwareMap);

        telemetry.addData("Status", "Initialized");

        // Send telemetry message to signify robot waiting;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        int inches;

        inches = 20;
        encoderDrive(1, inches, inches, inches, inches, 4);//Just Moves forward
        inches = 40;
        encoderDrive(1, -inches, inches, inches, -inches, 4);//Move Left

    }

    public void encoderDrive(double speed,
                             double fleftInches, double frightInches, double bleftInches, double brightInches,
                             double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            mecDrive.fleft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecDrive.fright_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecDrive.bleft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecDrive.bright_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            mecDrive.fleft_drive.setTargetPosition((int) (fleftInches * mecDrive.COUNTS_PER_INCH * mecDrive.Adjust));
            mecDrive.fright_drive.setTargetPosition((int) (frightInches * mecDrive.COUNTS_PER_INCH * mecDrive.Adjust));
            mecDrive.bleft_drive.setTargetPosition((int) (bleftInches * mecDrive.COUNTS_PER_INCH * mecDrive.Adjust));
            mecDrive.bright_drive.setTargetPosition((int) (brightInches * mecDrive.COUNTS_PER_INCH * mecDrive.Adjust));

            // Turn On RUN_TO_POSITION
            mecDrive.fleft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecDrive.fright_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecDrive.bleft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecDrive.bright_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            mecDrive.fleft_drive.setPower(Math.abs(speed)*(Math.abs(fleftInches)/fleftInches));
            mecDrive.fright_drive.setPower(Math.abs(speed)*(Math.abs(frightInches)/frightInches));
            mecDrive.bleft_drive.setPower(Math.abs(speed)*(Math.abs(bleftInches)/bleftInches));
            mecDrive.bright_drive.setPower(Math.abs(speed)*(Math.abs(brightInches)/brightInches));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecDrive.fleft_drive.isBusy() && mecDrive.fright_drive.isBusy()) && mecDrive.bleft_drive.isBusy() && mecDrive.bright_drive.isBusy()) {

            }

            // Stop all motion;
            mecDrive.fleft_drive.setPower(0);
            mecDrive.fright_drive.setPower(0);
            mecDrive.bleft_drive.setPower(0);
            mecDrive.bright_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            mecDrive.fleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecDrive.fright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecDrive.bleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecDrive.bright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
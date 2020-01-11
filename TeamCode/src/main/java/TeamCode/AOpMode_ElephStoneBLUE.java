package TeamCode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="AOpMode_ElephStoneBLUE", group="Linear Opmode")
//@Disabled
public class AOpMode_ElephStoneBLUE extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();
    private SubSys_MecDrive mecDrive = new SubSys_MecDrive();
    private SubSys_Elephant elephant = new SubSys_Elephant();





    @Override
    public void runOpMode() {


        mecDrive.init(hardwareMap);
        elephant.init(hardwareMap);

        telemetry.addData("Status", "Initialized");

        // Send telemetry message to signify robot waiting;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        elephant.eTail.setPosition(0.5);//Keep tail up, out of the way
        double inches;
        inches = 30;
        encoderDrive(1, inches, inches, inches, inches, 4);//Move Forward



        inches = 30;
        encoderDrive(1, inches, -inches, -inches, inches, 4);//Move Right
        inches = 45;
        encoderDrive(1, -inches, -inches, -inches, -inches, 4);//Move Back
        elephant.eTail.setPosition(0.95);//put Foundation grabber down
        sleep(1000);
        inches = 50;
        encoderDrive(1, inches, inches, inches, inches, 4);//Move Forward
        elephant.eTail.setPosition(0.5);//Release Tail
        inches = 30;
        encoderDrive(1, -inches, inches, inches, -inches, 4);//Move Left to get out from foundation
        inches = 2;
        encoderDrive(1, -inches, -inches, -inches, -inches, 2);//Move Back
        inches = 35;
        encoderDrive(1, -inches, inches, inches, -inches, 4);//Move Left to park under bridge
        inches = 5;
        encoderDrive(1, inches, inches, inches, inches, 2);//Move Forward

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

    public void encoderArm(double speed, int counts, double timeoutS) {

        telemetry.addLine();
        telemetry.addData("encoderArm", "(%.3f, %d, %.3f)", speed, counts, timeoutS);
        telemetry.update();

        if (opModeIsActive()) {
            elephant.eTrunk.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elephant.eTrunk.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // newServoTarget = (int) eNose.getPosition() + (int) (servo);

            telemetry.addData("Position", "current=%d new = %d", elephant.eTrunk.getCurrentPosition(), counts);
            telemetry.update();
            elephant.eTrunk.setTargetPosition(counts);
            // eNose.setPosition(newServoTarget);
            runtime.reset();
            elephant.eTrunk.setPower(speed);
            limit(speed);

            boolean opModeActive;
            double secs;
            boolean bBusy;
            do {
                opModeActive = opModeIsActive();
                secs = runtime.seconds();
                bBusy = elephant.eTrunk.isBusy();

                limit(speed);

                telemetry.addData("speed", speed);
                telemetry.update();


            } while(opModeActive && (secs < timeoutS) && bBusy);

            elephant.eTrunk.setPower(0);

            // Turn off RUN_TO_POSITION
            elephant.eTrunk.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elephant.eTrunk.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    private boolean limit(double desired_Speed){
        double output;
        if ((desired_Speed>0 && elephant.eTrunk.getCurrentPosition()<=elephant.maxTrunkHeight) || (desired_Speed<0 && elephant.eTrunk.getCurrentPosition()>=0)) {
            output = 0;
            elephant.eTrunk.setPower(output);
            telemetry.addData("limit", "*******stopping because of limit");
            return true;
        }
        return false;

    }
    private boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    }

}
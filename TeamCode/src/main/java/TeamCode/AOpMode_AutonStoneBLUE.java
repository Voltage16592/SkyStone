package TeamCode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="AOpMode_AutonStoneBLUE", group="Iterative Opmode")
@Disabled
public class AOpMode_AutonStoneBLUE extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor gNeck =  null;
    private Servo giraffeMouth = null;
    private DigitalChannel forwardLimitSwitch;
    private DigitalChannel reverseLimitSwitch;


    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265);
    static final double DRIVE_SPEED = 0.6;
    static final double Adjust = 1/9.52;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        gNeck = hardwareMap.get(DcMotor.class, "eTrunk");
        giraffeMouth = hardwareMap.get(Servo.class,  "eNose");
        forwardLimitSwitch = hardwareMap.get(DigitalChannel.class, "forwardLimitSwitch");
        reverseLimitSwitch = hardwareMap.get(DigitalChannel.class, "reverseLimitSwitch");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        gNeck.setDirection(DcMotor.Direction.FORWARD);
        giraffeMouth.setDirection(Servo.Direction.FORWARD);
        gNeck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setAutoClear(false);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gNeck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gNeck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        /*
        telemetry.addData("Path0", "Starting at %d %d %d %d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition(),
                eTrunk.getCurrentPosition(),
                eNose.getPosition());

        telemetry.update();
*/        // Wait for the game to start (driver presses PLAY)

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderArm(DRIVE_SPEED, 15, 2.0, 60);
        //encoderDrive(DRIVE_SPEED, 48, 48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        //sleep(1000);     // pause for servos to move

        //telemetry.addData("Path", "Complete");
        telemetry.addData("Start", "successful");
        telemetry.update();



        waitForStart();
        runtime.reset();






        encoderDrive(0.75, 2.1, 2.1, 4);//drive forward
        giraffeMouth.setPosition(0.75);//open mouth
        sleep(500);
        encoderArm(0.175,-900,3);//bring neck down
        sleep(3000);
        giraffeMouth.setPosition(0);//eat stone
        sleep(1000);
        encoderArm(-0.3,200,2);//lift neck up
        encoderDrive(0.75, -0.75, -0.75, 2);//Back up, parking on inside (away from walls)
        //encoderDrive(0.5, -3, -3, 4);//Back up outside, close to wall parking
        encoderDrive(0.75, -3.5, 3.5 , 2);//Turn left
        encoderArm(-0.3,200,2);//lift neck up
        encoderDrive(0.75, 1.25, 1.25, 4);//drive forward
        giraffeMouth.setPosition(0.75);//throw up
        encoderDrive(0.75, -0.25, -0.25, 2);//Back up


    }

    public void encoderArm(double speed, int counts, double timeoutS) {
        int newNeckTarget;

        telemetry.addLine();
        telemetry.addData("encoderArm", "(%.3f, %d, %.3f)", speed, counts, timeoutS);
        telemetry.update();

        if (opModeIsActive()) {
            gNeck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gNeck.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            newNeckTarget = gNeck.getCurrentPosition() + counts;
            // newServoTarget = (int) eNose.getPosition() + (int) (servo);

            RobotLog.d("test1", "this is a test");
            telemetry.addData("Position", "current=%d new = %d", gNeck.getCurrentPosition(), newNeckTarget);
            telemetry.update();
            gNeck.setTargetPosition(newNeckTarget);
            // eNose.setPosition(newServoTarget);
            runtime.reset();
            gNeck.setPower(speed);
            limit(speed);

            int iCount = 0;
            boolean opModeActive;
            double secs;
            boolean bBusy;
            do {
                iCount++;
                opModeActive = opModeIsActive();
                secs = runtime.seconds();
                bBusy = gNeck.isBusy();

                // Display it for the driver.
                limit(speed);
                /*
                telemetry.addData("Path1", "iCount=%d seconds()=%.3f",
                        iCount, secs);
                telemetry.update();
                telemetry.addData("Path1", "Running from %d to %d",
                        eTrunk.getCurrentPosition(), newNeckTarget);
                telemetry.update();

                telemetry.addData("finished", "iCount=%d opModeIsActive=%d, runtime.seconds()=%.3f, eTrunk.isBusy()=%d",
                       iCount, (opModeActive ? 1:0),secs,bBusy ? 1:0);
                */
                telemetry.addData("speed", speed);
                telemetry.update();


                } while(opModeActive && (secs < timeoutS) && bBusy);

            gNeck.setPower(0);

            // Turn off RUN_TO_POSITION
            gNeck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gNeck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int curLeftPos = leftDrive.getCurrentPosition();
            int curRightPos = rightDrive.getCurrentPosition();
            newLeftTarget = curLeftPos + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = curRightPos + (int) (rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double rightDriveSpeed = Math.abs(speed) * ((newRightTarget-curRightPos)/Math.abs(newRightTarget-curRightPos));
            double leftDriveSpeed = Math.abs(speed) * ((newLeftTarget-curLeftPos)/Math.abs(newLeftTarget-curLeftPos));
            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(leftDriveSpeed);
            rightDrive.setPower(rightDriveSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            /*
            while (opModeIsActive() &&

                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to ", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at ",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }
            */
            while (opModeIsActive() &&

                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {
                //waiting for timeout or to reach point
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    private boolean limit(double desired_Speed){
        double output;
        if ((desired_Speed>0 && isDetected(forwardLimitSwitch)) || (desired_Speed<0 && isDetected(reverseLimitSwitch))) {
            output = 0;
            gNeck.setPower(output);
            telemetry.addData("limit", "*******stopping because of limit");
            return true;
        }
        return false;

    }
    private boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    }
}

package TeamCode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import android.hardware.Camera;


import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name="AOpMode_AutonPull", group="Linear Opmode")
//@Disabled
public class AOpMode_AutonPull extends LinearOpMode {

    /* Declare OpMode members. */
    private Servo giraffeTail = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor gNeck =  null;
    private Servo giraffeMouth = null;
    private DigitalChannel forwardLimitSwitch;
    private DigitalChannel reverseLimitSwitch;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "Ado05xz/////AAABmY8uetMq2krthNRU8hk1XbAPBHbJ/EVJizmHI/8Kz+HV4an+0zONWUZOd9XOiJIebM2WA7z/Wzffa9W87IrMnmb4pKEkY5dYbzjEdsDy28aKcZSkAu7jpO610LnMv+tWDKK3Chj+apf7OinQiaMnm9xSdjIOTxe6kegt5kHTY6inImWrZuHXe6trOfv48elrDyhrTDNELqZwjjG1LFZkGzgyKCQ9wvWcO0JXec+R5iQg+RMc92eqhCMv/6558QRae364puvHtp0OfszOivgelgFk901BvjQzTFzYnh80+tFWbiNNGfc6jzyz09xcWBR9B9xOsIPHcNPgsF9akWHjrEaDCtj/XdsrlqOf93xq31fl ";

    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159267);
    static final double DRIVE_SPEED = 0.6;
    static final double Adjust = 1/9.52;
    static final double TURN_SPEED = 0.8;
    private VuforiaLocalizer vuforia = null;

    public static Camera cam = null;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;



        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.



        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");



        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        giraffeTail = hardwareMap.get(Servo.class, "eTail");
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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gNeck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gNeck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at ",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition(),
                gNeck.getCurrentPosition(),
                giraffeMouth.getPosition());

        telemetry.update();


        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);








        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        giraffeTail.setPosition(0.5);
        encoderDrive(DRIVE_SPEED, -35, -35, 4);
        giraffeTail.setPosition(0.1);
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 32, 32, 10);
        giraffeTail.setPosition(0);
        giraffeTail.setPosition(0.5);

    }

    public void encoderArm(double speed, double degrees, double timeoutS, double servo) {
        int newUpTarget;
        int newServoTarget;

        if (opModeIsActive()) {
            newUpTarget = gNeck.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            newServoTarget = (int) giraffeMouth.getPosition() + (int) (servo);
            gNeck.setTargetPosition(newUpTarget);
            limit();
            giraffeMouth.setPosition(newServoTarget);
            gNeck.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            gNeck.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (gNeck.isBusy())) {

                // Display it for the driver.
                limit();
                telemetry.addData("Path1", "Running to ", newUpTarget);
                telemetry.addData("Path2", "Running at ",
                        gNeck.getCurrentPosition());
                telemetry.update();
            }

            gNeck.setPower(0);

            // Turn off RUN_TO_POSITION
            gNeck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH * Adjust);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH * Adjust);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
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

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    private void limit(){
        double output;
        if (!isDetected(forwardLimitSwitch) || !isDetected(reverseLimitSwitch)) {
            output = 0;
            gNeck.setPower(output);
        }

    }
    private boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    }
}
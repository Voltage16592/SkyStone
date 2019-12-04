/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package TeamCode;

import android.hardware.Camera;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name=" AOp_AutonStoneVufMec", group="Linear Opmode")
public class AOp_AutonStoneVufMec extends LinearOpMode {

    private static final String VUFORIA_KEY =
            "Ado05xz/////AAABmY8uetMq2krthNRU8hk1XbAPBHbJ/EVJizmHI/8Kz+HV4an+0zONWUZOd9XOiJIebM2WA7z/Wzffa9W87IrMnmb4pKEkY5dYbzjEdsDy28aKcZSkAu7jpO610LnMv+tWDKK3Chj+apf7OinQiaMnm9xSdjIOTxe6kegt5kHTY6inImWrZuHXe6trOfv48elrDyhrTDNELqZwjjG1LFZkGzgyKCQ9wvWcO0JXec+R5iQg+RMc92eqhCMv/6558QRae364puvHtp0OfszOivgelgFk901BvjQzTFzYnh80+tFWbiNNGfc6jzyz09xcWBR9B9xOsIPHcNPgsF9akWHjrEaDCtj/XdsrlqOf93xq31fl ";
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private VuforiaLocalizer vuforia = null;
    public static Camera cam = null;
    private DcMotor gNeck =  null;
    private Servo giraffeMouth = null;DcMotor fleft_drive;//front left motor
    private DigitalChannel forwardLimitSwitch;DcMotor fright_drive;//front right motor
    private DigitalChannel reverseLimitSwitch;DcMotor bleft_drive;//back left motor
    DcMotor bright_drive;//back left motor
    double fleft_multiplier = 0.865;
    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159267);
    static final double DRIVE_SPEED = 0.6;
    static final double Adjust = 1/9.52;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {


            fleft_drive = hardwareMap.get(DcMotor.class, "fleft_drive");
            fleft_drive.setDirection(DcMotor.Direction.REVERSE);
            fright_drive = hardwareMap.get(DcMotor.class, "fright_drive");
            fright_drive.setDirection(DcMotor.Direction.FORWARD);
            bleft_drive = hardwareMap.get(DcMotor.class, "bleft_drive");
            bleft_drive.setDirection(DcMotor.Direction.REVERSE);
            bright_drive = hardwareMap.get(DcMotor.class, "bright_drive");
            bright_drive.setDirection(DcMotor.Direction.FORWARD);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;

            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parametersV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parametersV.vuforiaLicenseKey = VUFORIA_KEY;
            parametersV.cameraDirection   = CAMERA_CHOICE;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parametersV);

            // Load the data sets for the trackable objects. These particular data
            // sets are stored in the 'assets' part of our application.
            VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

            VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
            stoneTarget.setName("Stone Target");
            telemetry.addData("Status:", "Initialized");
            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                        sleep(50);
                        idle();
            }
            telemetry.addData("Mode", "waiting for start");
            telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());telemetry.update();
            telemetry.update();
        waitForStart();
        double D1 = 10
                ; // move halfway between stone and wall
        double D2 = 8.78; // once sky stone found, move to sky stone
        double D3 = 13+8.78; // move under the bridge to drop stone
        double D4 = 24; // move under the bridge
        encoderDrive(0.8,D1, D1, 4000, false);
        /*
        while (!((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
            setMotorPowerAll(-0.5 * fleft_multiplier,0.5, 0.5, -0.5);
            telemetry.addData("Visible Target", "None");
            telemetry.update();
        }
        */
        telemetry.addData("Visible Target", "SkyStone");
        telemetry.update();
        //encoderDrive(1.0,D2, D2, 4000, false);
        //pickup
        //encoderDrive(1.0,-D3, -D3, 4000, false);
        //rotate(-90, 1.0);
        //encoderDrive(1.0,D4, D4, 4000, false);
        //setdown
        //encoderDrive(1.0,-D4, -D4, 4000, false);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("imu heading", lastAngles.firstAngle);
            telemetry.update();
        }
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, boolean Sideways) {
        int newLeftTarget;
        int newRightTarget;
        int y = 1;
        if (Sideways == true){
           y = -1;
        }
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = fleft_drive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = fright_drive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            fleft_drive.setTargetPosition(newLeftTarget);
            fright_drive.setTargetPosition(newRightTarget);
            bleft_drive.setTargetPosition(newLeftTarget);
            bright_drive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            fleft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fright_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bleft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bright_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fleft_drive.setPower(Math.abs(speed)*y);
            fright_drive.setPower(Math.abs(speed));
            bleft_drive.setPower(Math.abs(speed));
            bright_drive.setPower(Math.abs(speed)*y);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will  lok
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fleft_drive.isBusy() && fright_drive.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to ", newLeftTarget, newRightTarget);
                //telemetry.addData("Path2", "Running at ",
                        fleft_drive.getCurrentPosition();
                        fright_drive.getCurrentPosition();
                //telemetry.update();
            }

            // Stop all motion;
            fleft_drive.setPower(0);
            fright_drive.setPower(0);
            bright_drive.setPower(0);
            bleft_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            bright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    private double ramp_Motor_Power(double current_Power, double desired_Power){
        double diff = desired_Power-current_Power;
        if(Math.abs(desired_Power) <0.2 || Math.abs(diff)<0.05)
            current_Power = desired_Power;
        else
            current_Power += (diff/(Math.abs(diff)))*0.05;
        return current_Power;
    }

     private void setMotorPowerAll(double fl, double fr, double bl, double br) {
        fleft_drive.setPower(ramp_Motor_Power(fleft_drive.getPower(), fl)*fleft_multiplier);
        fright_drive.setPower(ramp_Motor_Power(fright_drive.getPower(), fr));
        bleft_drive.setPower(ramp_Motor_Power(bleft_drive.getPower(), bl));
        bright_drive.setPower(ramp_Motor_Power(bright_drive.getPower(), br));
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        fleft_drive.setPower(leftPower*fleft_multiplier);
        fleft_drive.setPower(leftPower);
        bright_drive.setPower(rightPower);
        fright_drive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        fright_drive.setPower(0);
        bright_drive.setPower(0);
        fleft_drive.setPower(0);
        bleft_drive.setPower(0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;


        lastAngles = angles;
        return deltaAngle;
    }



}

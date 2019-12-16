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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name=" AOp_AutonStoneVufMec", group="Linear Opmode")
public class AOp_AutonStoneVufMec extends LinearOpMode {

   private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU               imu;
    double globalAngle = 0;
    Orientation             lastAngles = new Orientation();
    private DcMotor gNeck =  null;
    private Servo giraffeMouth = null;
    //DcMotor fleft_drive;//front left motor
    private DigitalChannel forwardLimitSwitch;
    //DcMotor fright_drive;//front right motor
    private DigitalChannel reverseLimitSwitch;
    //DcMotor bleft_drive;//back left motor
    //DcMotor bright_drive;//back left motor
    double fleft_multiplier = 0.91;
    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159267);
    static final double DRIVE_SPEED = 0.6;
    static final double Adjust = 1/9.52;
    static final double TURN_SPEED = 0.5;
    private SubSys_MecDrive mecDrive = new SubSys_MecDrive();
    private Subsys_Vuforia vuforiaSys = new Subsys_Vuforia();
    private Subsys_gyroscope gyroscope = new Subsys_gyroscope();


    @Override
    public void runOpMode() {

            telemetry.update();
        mecDrive.init(hardwareMap);
        telemetry.addData("MecDrive:", "Initialized");
        vuforiaSys.runOpMode(hardwareMap);
        telemetry.addData("Vuforia:", "Initialized");
        gyroscope.init(hardwareMap);
        telemetry.addData("Gyroscope:","Calibrated");
        telemetry.update();
        waitForStart();




        double D1 = 8; // move halfway between stone and wall
        double D2 = 8.78; // once sky stone found, move to sky stone
        double D3 = 21.78; // move under the bridge to drop stone
        double D4 = 24; // move under the bridge
        encoderDrive(1.0, 35, 35, 4000, false);
        /*
        encoderDrive(1.0,D1, D1, 4000, false);
        encoderDrive(1.0,D2, D2, 4000, false)
        //pickup
        encoderDrive(1.0, -12, -12, 4000, false);
        rotate(74, 1.0);
        //encoderDrive(1.0,-D3, -D3, 4000, false);

        encoderDrive(1.0,D4, D4, 4000, false);
        //setdown
        encoderDrive(1.0,-D4, -D4, 4000, false);
        */
        /*
        telemetry.addData("Visible Target", "None");

        vuforiaSys.detect();
        telemetry.addData("Visible Target", "SkyStone");
        telemetry.update();
        runtime.reset();
        */

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("imu", gyroscope.getAngle());
            telemetry.addData("bright pos", mecDrive.bright_drive.getCurrentPosition());
            telemetry.addData("bleft pos", mecDrive.bleft_drive.getCurrentPosition());
            telemetry.addData("fright pos", mecDrive.fright_drive.getCurrentPosition());
            telemetry.addData("fleft pos", mecDrive.fleft_drive.getCurrentPosition());
            telemetry.update();
        }
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, boolean Sideways) {

        int newLeftTarget;
        int newRightTarget;
        int y = 1;
        double adjust = 0.15;
        leftInches *= adjust;
        rightInches *= adjust;
        double fleft_multiplier1 = fleft_multiplier;
        if (Sideways == true){
           y = -1;
           fleft_multiplier1 = 1;
        }
        double pow = Math.abs(speed);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            //reset all encoders
            mecDrive.fleft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecDrive.fright_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecDrive.bleft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecDrive.bright_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            mecDrive.bright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecDrive.fright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecDrive.bleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecDrive.fleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Determine new target position, and pass to motor controller

            newLeftTarget = /*mecDrive.fleft_drive.getCurrentPosition() + */(int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = /*mecDrive.fright_drive.getCurrentPosition() + */(int) (rightInches * COUNTS_PER_INCH);
            /*
            mecDrive.fleft_drive.setTargetPosition(newLeftTarget*y);
            mecDrive.fright_drive.setTargetPosition(newRightTarget);
            mecDrive.bleft_drive.setTargetPosition(newLeftTarget);
            mecDrive.bright_drive.setTargetPosition(newRightTarget*y);
        */
            double increment = speed/10;
            int ALP = newLeftTarget-(int)(0.5*speed)* (int) (0.182+0.25);
            int ARP = newRightTarget-(int)(0.5*speed)* (int) (0.182+0.25);
            mecDrive.fleft_drive.setTargetPosition(ALP*y);
            mecDrive.fright_drive.setTargetPosition(ARP);
            mecDrive.bleft_drive.setTargetPosition(ALP);
            mecDrive.bright_drive.setTargetPosition(ARP*y);


            telemetry.addData("bright pos", mecDrive.bright_drive.getCurrentPosition());
            telemetry.addData("bleft pos", mecDrive.bleft_drive.getCurrentPosition());
            telemetry.addData("fright pos", mecDrive.fright_drive.getCurrentPosition());
            telemetry.addData("fleft pos", mecDrive.fleft_drive.getCurrentPosition());
            telemetry.update();

            // Turn On RUN_TO_POSITION
            mecDrive.fleft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecDrive.fright_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecDrive.bleft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecDrive.bright_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion

            double currentPower = 0;
            runtime.reset();
            while(currentPower < speed) {
                mecDrive.fleft_drive.setPower(Math.abs(currentPower) * fleft_multiplier1 * y);
                mecDrive.fright_drive.setPower(Math.abs(currentPower));
                mecDrive.bleft_drive.setPower(Math.abs(currentPower));
                mecDrive.bright_drive.setPower(Math.abs(currentPower) * y);
                currentPower += increment;
                telemetry.addData("currentPower", currentPower);
                telemetry.addData("Desired Power:", speed);
                telemetry.update();
                sleep(25);
            }

            double time = runtime.seconds();



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will  lok
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecDrive.fleft_drive.isBusy() && mecDrive.fright_drive.isBusy()&& mecDrive.bleft_drive.isBusy() && mecDrive.bright_drive.isBusy())) {
            }
            setMotorPowerAll(speed, speed, speed, speed);
            boolean did = false;
            while(currentPower > 0) {
                mecDrive.fleft_drive.setPower(Math.abs(currentPower) * fleft_multiplier1 * y);
                mecDrive.fright_drive.setPower(Math.abs(currentPower));
                mecDrive.bleft_drive.setPower(Math.abs(currentPower));
                mecDrive.bright_drive.setPower(Math.abs(currentPower) * y);
                currentPower -= increment;
                telemetry.addData("currentPower", currentPower);
                telemetry.addData("Desired Power:", 0);
                telemetry.update();
                sleep(25);
                did = true;
            }
            telemetry.addData("did", did);
            telemetry.update();

            // Stop all motion;
            mecDrive.fleft_drive.setPower(0);
            mecDrive.fright_drive.setPower(0);
            mecDrive.bright_drive.setPower(0);
            mecDrive.bleft_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            mecDrive.bright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecDrive.fright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecDrive.bleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mecDrive.fleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
         mecDrive.fleft_drive.setPower(fl*fleft_multiplier);
         mecDrive.fright_drive.setPower(fr);
         mecDrive.bleft_drive.setPower(bl);
         mecDrive.bright_drive.setPower(br);
    }
    /*
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    */


    public void encoderArm(double speed, double degrees, double timeoutS, double servo) {
        int newUpTarget;
        int newServoTarget;

        if (opModeIsActive()) {
            newUpTarget = gNeck.getCurrentPosition() + (int) (degrees);
            // newServoTarget = (int) eNose.getPosition() + (int) (servo);
            gNeck.setTargetPosition(newUpTarget);
            limit(speed);
            // eNose.setPosition(newServoTarget);
            gNeck.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            gNeck.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (gNeck.isBusy())) {

                // Display it for the driver.
                limit(speed);
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
    private boolean limit(double desired_Speed){
        double output;
        if ((desired_Speed<0 && isDetected(forwardLimitSwitch)) || (desired_Speed>0 && isDetected(reverseLimitSwitch))) {
            output = 0;
            gNeck.setPower(output);
            telemetry.addData("limit", "*******stopping because of limit");
            return true;
        }
        return false;

    }

    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        gyroscope.resetAngle();

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
        mecDrive.fleft_drive.setPower(leftPower*fleft_multiplier);
        mecDrive.fleft_drive.setPower(leftPower);
        mecDrive.bright_drive.setPower(rightPower);
        mecDrive.fright_drive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && gyroscope.getAngle() == 0) {}

            while (opModeIsActive() && gyroscope.getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && gyroscope.getAngle() < degrees) {}

        // turn the motors off.
        mecDrive.fright_drive.setPower(0);
        mecDrive.bright_drive.setPower(0);
        mecDrive.fleft_drive.setPower(0);
        mecDrive.bleft_drive.setPower(0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        gyroscope.resetAngle();
    }

    private boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    }


}

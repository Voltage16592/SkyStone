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
    double fleft_multiplier = 0.89;
    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring ycircumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159267);
    static final double DRIVE_SPEED = 0.6;
    static final double Adjust = 0.43;
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
        encoderDrive(1.0, 47, 47, 4000, false);

        double time = 0;
        //        //goes right until detects SkyStone and then stops
        //detects at range of 9 in, try not to make further
        /*
        runtime.reset();
        encoderDrive(1.0,25.5,25.5, 4000, false);
        setMotorPowerAll(0.25, -0.25, -0.25, 0.25);
        vuforiaSys.detect();
        time = getRuntime();
        setMotorPowerAll(0,0,0,0);

        encoderDrive(1.0, 5.0, 5.0, 4000, false);
        encoderArm(1.0, 360, 2000,  0);
        encoderDrive(1.0, -10, -10, 4000, false);
        rotate(-74, 1.0);
        runtime.reset();
        setMotorPowerAll(1.0,1.0,1.0,1.0);
        while(getRuntime() < time){}
        encoderDrive(1.0, 31, 31, 5000, false);
        encoderDrive(1.0, -47, -47, 4000, false);
        runtime.reset();
        setMotorPowerAll(-1.0,-1.0,-1.0,-1.0);
        while(getRuntime() < time){

        }
        setMotorPowerAll(0,0,0,0);
        rotate(74, 1.0);
        encoderDrive(1.0, 10, 10, 3000, false);
        encoderArm(1.0, 360, 2000,  0);
        encoderDrive(1.0, -10, -10, 4000, false);
        encoderArm(1.0, 360, 4000, 0);
        rotate(-74, 1.0);

        setMotorPowerAll(-1.0,-1.0,-1.0,-1.0);
        */
        while(getRuntime() < time){

        }

            // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetry.addData("imu", gyroscope.getAngle());
            //telemetry.addData("bright pos", mecDrive.bright_drive.getCurrentPosition());
            //telemetry.addData("bleft pos", mecDrive.bleft_drive.getCurrentPosition());
            //telemetry.addData("fright pos", mecDrive.fright_drive.getCurrentPosition());
            //telemetry.addData("fleft pos", mecDrive.fleft_drive.getCurrentPosition());
            //telemetry.update();
        }
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, boolean Sideways) {

        int newLeftTarget;
        int newRightTarget;
        int newfLeftTarget;
        int y = 1;
        double adjust = 0.15;
        leftInches *= adjust;
        rightInches *= adjust;
        if (Sideways == true){
           y = -1;
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

            newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
            newfLeftTarget = (int) (leftInches*fleft_multiplier*COUNTS_PER_INCH);

            mecDrive.fleft_drive.setTargetPosition(newfLeftTarget*y);
            mecDrive.fright_drive.setTargetPosition(newRightTarget);
            mecDrive.bleft_drive.setTargetPosition(newLeftTarget);
            mecDrive.bright_drive.setTargetPosition(newRightTarget*y);

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



            double currentPower = 0;

                while(currentPower < speed){
                setMotorPowerAll(currentPower, currentPower, currentPower, currentPower);
                currentPower += speed/10;
                sleep(25);
            }


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecDrive.fleft_drive.isBusy() || mecDrive.fright_drive.isBusy()|| mecDrive.bleft_drive.isBusy() || mecDrive.bright_drive.isBusy())) {
            }

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

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
@Autonomous(name=" AOp_AutonStoneVufMecBLUE", group="Linear Opmode")
public class AOp_AutonStoneVufMecBLUE extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU               imu;
    double globalAngle = 0;
    Orientation             lastAngles = new Orientation();
    private DcMotor gNeck =  null;
    double fleft_multiplier = 0.89;
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
    private SubSys_Elephant Elephant = new SubSys_Elephant();
    @Override
    public void runOpMode() {
        telemetry.update();
        mecDrive.init(hardwareMap);
        telemetry.addData("MecDrive:", "Initialized");
        vuforiaSys.runOpMode(hardwareMap);
        telemetry.addData("Vuforia:", "Initialized");
        gyroscope.init(hardwareMap);
        Elephant.init(hardwareMap);
        telemetry.addData("Gyroscope:","Calibrated");
        telemetry.update();
        waitForStart();

        Elephant.eNose.setPosition(0);

        encoderDrive(1.0, 23.5,  1.5, false);    //drive forward
        //telemetry.addData("time", runtime.seconds());
        //telemetry.update();
        boolean detect = false;
        double time = 0;
        runtime.reset();
        setMotorPowerAll(0.26, -0.24, -0.26, 0.24); //strafing, looking for skystone
        vuforiaSys.detect();
        time=runtime.seconds();
        detect = true;
        //telemetry.addData("time", runtime.seconds());
        //telemetry.update();
        setMotorPowerAll(0,0,0,0);  //stop once detected
        encoderDrive(1, -8,  1, true);   //move more right to line skystone up w lift, not phone
        //rotate(-1, 0.25);
        encoderDrive(1, 5,  1, false);    //move forward to pick up stone
        Elephant.eNose.setPosition(1);  //pick up stone
        sleep(1000);
        encoderDrive(1, -10, 1.3, false);    //back up
        reAlign();
        //rotate(66, 1);  //turn towards bridge
        encoderDrive(1.0, 50, 3, true);    //drive under bridge to deliver stone

        Elephant.eNose.setPosition(0);  //release stone
        rotate(8, 0.25, 0.5);
        encoderDrive(1, -70,  2.5 + time/4, true);    //drive back to stones
        telemetry.addData("rotating", "yes");
        reAlign();
//        rotate(35, 0.75, 1.3);
        encoderDrive(1,10,1.3 , false);
        Elephant.eNose.setPosition(1);
        sleep(500);
        encoderDrive(1, -10, 1.3, false);

        encoderDrive(1, 70, 2.5  + time/4, true);

        //rotate(-35, 0.75, 1.3);
        Elephant.eNose.setPosition(0.5);
        encoderDrive(1, -20, 1, true);




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("imu", gyroscope.getAngle());
            //telemetry.addData("bright pos", mecDrive.bright_drive.getCurrentPosition());
            //telemetry.addData("bleft pos", mecDrive.bleft_drive.getCurrentPosition());
            //telemetry.addData("fright pos", mecDrive.fright_drive.getCurrentPosition());
            //telemetry.addData("fleft pos", mecDrive.fleft_drive.getCurrentPosition());
            //telemetry.addData("skystone detected: " , detect);
            telemetry.update();
        }
    }
    public void encoderDrive(double speed,
                             double distance,
                             double timeoutS, boolean Sideways) {

        int target;
        int y = 1;
        double adjust = 0.15;
        distance *= adjust;
        if (Sideways == true){
           y = -1;
        }
        double pow = Math.abs(speed);
        // Ensure that the opmode is still active
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {


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

            target = (int) (distance * COUNTS_PER_INCH);
            double a = 1;
            double b = 1;
            double c = 1;
            double d = 1;
            if(Sideways == true){
                a = 0.8;
                b = 1;
                c = 0.8;
                d = 1;
            }

            mecDrive.fleft_drive.setTargetPosition((int)((target*y*fleft_multiplier)));
            mecDrive.fright_drive.setTargetPosition(target);
            mecDrive.bleft_drive.setTargetPosition((int)(target));
            mecDrive.bright_drive.setTargetPosition(target*y);

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



            setMotorPowerAll(speed*a, speed*b, speed*c, speed*d);

            /*
            double currentPower = 0;

                while(currentPower < speed){
                setMotorPowerAll(currentPower, currentPower, currentPower, currentPower);
                currentPower += speed/10;
                sleep(25);
            }
            */


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecDrive.fleft_drive.isBusy() && mecDrive.fright_drive.isBusy()&& mecDrive.bleft_drive.isBusy() & mecDrive.bright_drive.isBusy())) {
            }

            setMotorPowerAll(0, 0, 0, 0);

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



    public void reAlign(){
        int y = 1;
        if(globalAngle < 0) {
            y = -1;
        }
        setMotorPowerAll(y*0.05, -y*0.05, y*0.05, -y*0.05);
        if (y > 0)
        {
            // On right turn we have to get off zero first.
//            while (opModeIsActive() && gyroscope.getAngle() == 0) {}

            while (opModeIsActive() && gyroscope.getAngle() > 0) {}
        }
        else    // left turn.
            while (opModeIsActive() && gyroscope.getAngle() < 0) {}

        // turn the motors off.
        setMotorPowerAll(0, 0, 0, 0);


    }

    public void rotate(int degrees, double power, double timeoutS)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.

            gyroscope.resetAngle();


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if(degrees == 0)
            return;

        // set power to rotate.
        setMotorPowerAll(-power*(Math.abs(degrees)/degrees), power*(Math.abs(degrees)/degrees), -power*(Math.abs(degrees)/degrees), power*(Math.abs(degrees)/degrees));

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && gyroscope.getAngle() == 0) {}

            while (opModeIsActive() && gyroscope.getAngle() > degrees && (runtime.seconds() < timeoutS)) {}
        }
        else    // left turn.
            while (opModeIsActive() && gyroscope.getAngle() < degrees &&(runtime.seconds() < timeoutS)) {}

        // turn the motors off.
        setMotorPowerAll(0, 0, 0, 0);


        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        gyroscope.resetAngle();
    }

    private boolean isDetected(DigitalChannel limitSwitch) {
        return !limitSwitch.getState();
    }


}

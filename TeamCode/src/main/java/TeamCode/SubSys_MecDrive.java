package TeamCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
public class SubSys_MecDrive {
    DcMotor fleft_drive;//front left motor
    DcMotor fright_drive;//front right motor
    DcMotor bleft_drive;//back left motorh

    DcMotor bright_drive;//back left motor
    double fleft_multiplier = 0.91;
    HardwareMap hardwareMap;
    //telemetry.addData("fuck you");
    //for encoder driving:
    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.14159265);

    SubSys_MecDrive(){}

    public void init(HardwareMap hM){
        this.hardwareMap = hM;
        fleft_drive = hardwareMap.get(DcMotor.class, "fleft_drive");
        fleft_drive.setDirection(DcMotor.Direction.REVERSE);
        fright_drive = hardwareMap.get(DcMotor.class, "fright_drive");
        fright_drive.setDirection(DcMotor.Direction.FORWARD);
        bleft_drive = hardwareMap.get(DcMotor.class, "bleft_drive");
        bleft_drive.setDirection(DcMotor.Direction.REVERSE);
        bright_drive = hardwareMap.get(DcMotor.class, "bright_drive");
        bright_drive.setDirection(DcMotor.Direction.FORWARD);

    }

    public void move(double fwd_bkwd, double rt_lt, double clockwise_speed, double counterClockwise_speed){


        if ((fwd_bkwd <-0.25 || fwd_bkwd>0.25) && rt_lt >-0.25 && rt_lt <0.25 ) {
            setMotorPowerAll(fwd_bkwd,fwd_bkwd, fwd_bkwd, fwd_bkwd);
        }   //Moves motor forward and backward

        if ((rt_lt <-0.25 || rt_lt >0.25) && fwd_bkwd >-0.25 && fwd_bkwd<0.25){
            setMotorPowerAll(-rt_lt,rt_lt, rt_lt, -rt_lt);
        }   //Moves robot side to side (strafe)

        if ((fwd_bkwd > 0.25 && rt_lt < -0.25) || (fwd_bkwd < -0.25 && rt_lt > 0.25)) {
            setMotorPowerAll((-rt_lt + fwd_bkwd)*2/3,0, 0, (-rt_lt + fwd_bkwd)*2/3);
        }   //this moves the robot at a 45 degree angle (hence the number 45 in telemetry)

        if ((fwd_bkwd > 0.25 && rt_lt > 0.25) || (fwd_bkwd < -0.25 && rt_lt < -0.25)) {
            setMotorPowerAll(0,(rt_lt + fwd_bkwd)*2/3,(rt_lt + fwd_bkwd)*2/3, 0 );

        }   //this moves the robt at a 135 degree angle (hence the number 135 in telemetry)

        if (fwd_bkwd<0.25 && fwd_bkwd>-0.25 && rt_lt<0.25 && rt_lt>-0.25 && clockwise_speed<0.1 && counterClockwise_speed<0.1) {
            setMotorPowerAll(0,0, 0, 0);
        }   //When no buttons are pressed, all motors stop

        if (clockwise_speed>0) {         //right bumper makes the robot spin clockwise
            setMotorPowerAll(clockwise_speed,-clockwise_speed, clockwise_speed, -clockwise_speed);
        }else if (counterClockwise_speed>0) {    //left bumper makes the robot spin counterclockwise
            setMotorPowerAll(-counterClockwise_speed,counterClockwise_speed, -counterClockwise_speed, counterClockwise_speed);
        }
    }


    private void setMotorPowerAll(double fl, double fr, double bl, double br) {
        fleft_drive.setPower(ramp_Motor_Power(fleft_drive.getPower(), fl)*fleft_multiplier);
        fright_drive.setPower(ramp_Motor_Power(fright_drive.getPower(), fr));
        bleft_drive.setPower(ramp_Motor_Power(bleft_drive.getPower(), bl));
        bright_drive.setPower(ramp_Motor_Power(bright_drive.getPower(), br));
    }


    private double ramp_Motor_Power(double current_Power, double desired_Power){
        double diff = desired_Power-current_Power;
        if(Math.abs(desired_Power) <0.2 || Math.abs(diff)<0.05)
            current_Power = desired_Power;
        else
            current_Power += (diff/(Math.abs(diff)))*0.05;
        return current_Power;
    }


    }




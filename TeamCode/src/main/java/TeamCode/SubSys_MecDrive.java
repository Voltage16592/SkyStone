package TeamCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubSys_MecDrive {
    DcMotor fleft_drive;//front left motor
    DcMotor fright_drive;//front right motor
    DcMotor bleft_drive;//back left motor
    DcMotor bright_drive;//back left motor
    HardwareMap hardwareMap;

    SubSys_MecDrive(){}

    public void init(HardwareMap hM){
        this.hardwareMap = hM;
        fleft_drive = hardwareMap.get(DcMotor.class, "fleft_drive");
        fright_drive = hardwareMap.get(DcMotor.class, "fright_drive");
        bleft_drive = hardwareMap.get(DcMotor.class, "bleft_drive");
        bright_drive = hardwareMap.get(DcMotor.class, "bright_drive");
    }

    public void move(double fwd_bkwd, double rt_lt, double clockwise_speed, double counterClockwise_speed){
        if ((fwd_bkwd <-0.25 || fwd_bkwd>0.25) &&rt_lt >-0.25 && rt_lt <0.25 ) {
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
        fleft_drive.setPower(ramp_Motor_Power(fleft_drive.getPower(), fl));
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

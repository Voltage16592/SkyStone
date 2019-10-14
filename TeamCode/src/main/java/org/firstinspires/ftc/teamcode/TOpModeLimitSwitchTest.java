package org.firstinspires.ftc.teamcode;
package edu.wpi.first.wpilibj.templates;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import edu.wpi.first.wpilib.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;


@TeleOp(name="LimitSwitchTest", group="Iterative Opmode")

public class LimitSwitchTest extends IterativeRobot {
    DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    Talon motor;
    Joystick joystick1;

    public void robotInit() {
        DigitalInput forwardLimitSwitch = new DigitalInput(1);
        DigitalInput reverseLimitSwitch = new DigitalInput(2);
        Talon motor = new Talon(1);
        Joystick joystick1 = new Joystick(1);
    }


    public void teleopPeriodic() {
        int output = joystick1.getY(); //Moves the joystick based on Y value
        if (forwardLimitSwitch.get()) // If the forward limit switch is pressed, we want to keep the values between -1 and 0
            output = Math.min(output, 0);
        else if (reverseLimitSwitch.get()) // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            output = Math.max(output, 0);
        motor.set(output);
    }

}

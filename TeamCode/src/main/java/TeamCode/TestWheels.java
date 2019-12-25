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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TestWheels", group="Linear Opmode")
public class TestWheels extends LinearOpMode {

    private SubSys_MecDrive mecDrive = new SubSys_MecDrive();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        mecDrive.init(hardwareMap);
        telemetry.addData("MecDrive:", "Initialized");
        telemetry.update();
        waitForStart();
        mecDrive.bright_drive.setPower(1.0);
        telemetry.addData("motor", "back right");
        telemetry.update();
        sleep(10000);
        mecDrive.bright_drive.setPower(0);
        mecDrive.bleft_drive.setPower(1.0);
        telemetry.addData("motor", "back left");
        telemetry.update();
        sleep(10000);
        mecDrive.bleft_drive.setPower(0);
        runtime.reset();
        mecDrive.fright_drive.setPower(1.0);
        telemetry.addData("motor", "frong right");
        telemetry.update();
        sleep(10000);
        mecDrive.fright_drive.setPower(0);
        mecDrive.fleft_drive.setPower(1.0);
        telemetry.addData("motor", "front left");
        telemetry.update();
        sleep(10000);
        mecDrive.fleft_drive.setPower(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }
}

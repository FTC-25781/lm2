/* Copyright (c) 2024 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/*
 * This OpMode demonstrates how to use a digital channel.
 *
 * The OpMode assumes that the digital channel is configured with a name of "digitalTouch".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Limit Switch Test", group = "Sensor")

public class LimitSwitchTest extends LinearOpMode {
    DigitalChannel depositLmtSw;  // Digital channel Object
    DigitalChannel intakeLmtSw;  // Digital channel Object
    private DcMotor hsmot = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {

        // get a reference to our touchSensor object.
        depositLmtSw = hardwareMap.get(DigitalChannel.class, "dpltsw");
        intakeLmtSw = hardwareMap.get(DigitalChannel.class, "inltsw");

        depositLmtSw.setMode(DigitalChannel.Mode.INPUT);
        intakeLmtSw.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("DigitalTouchSensorExample", "Press start to continue...");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hsmot  = hardwareMap.get(DcMotor.class, "hsmot");
//        rightDrive = hardwareMap.get(DcMotor.class, "vsmot2");
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        hsmot.setDirection(DcMotor.Direction.FORWARD);
        hsmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // wait for the start button to be pressed.
        waitForStart();

        // while the OpMode is active, loop and read the digital channel.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // button is pressed if value returned is LOW or false.
            // send the info back to driver station using telemetry function.
            if (depositLmtSw.getState() == false) {
                telemetry.addData("Dp Button", "PRESSED");
            } else {
                telemetry.addData("Dp Button", "NOT PRESSED");
            }
            if (intakeLmtSw.getState() == false) {
                telemetry.addData("In Button", "PRESSED");
            } else {
                telemetry.addData("In Button", "NOT PRESSED");
            }
            telemetry.update();

            double power = -gamepad1.left_stick_y;
            // when limit sw is hit
            if(!depositLmtSw.getState() && power<0) {
                hsmot.setPower(power);
            } else if (depositLmtSw.getState()) {
                hsmot.setPower(power);
            }

            if(!depositLmtSw.getState()) {
                hsmot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hsmot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(gamepad1.a) {
                hsmot.setTargetPosition(-4994);
                hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hsmot.setPower(-0.2);
                while (hsmot.isBusy() ) {
                    telemetry.addData("Currently at",  " at %7d", hsmot.getCurrentPosition());
                    telemetry.update();
                }
                hsmot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


//            rightDrive.setPower(power);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "power (%.2f)", power);
            telemetry.addData("motor position", hsmot.getCurrentPosition());
            telemetry.update();
        }
    }
}

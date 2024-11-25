
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Motor Direction Test", group="Linear OpMode")
public class MotorDirection extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "hsmot");
//        rightDrive = hardwareMap.get(DcMotor.class, "vsmot2");
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            leftDrive.setPower(power);
//            rightDrive.setPower(power);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "power (%.2f)", power);
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Match Teleop", group = "Teleop")
public class MatchTeleOp extends LinearOpMode {

    // Robot components
    private Robot robot;
    private MecanumDrive drive;
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Orientation tracking
    private double orientationPosition = 0.0;

    @Override
    public void runOpMode() {
        // Initialize robot and drive system
        robot = new Robot(hardwareMap, telemetry);
        initializeMotors();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            // Set drive powers based on gamepad input
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x
            ));

            // Update the robot's pose estimate
            drive.updatePoseEstimate();

            // Update subsystems
            updateIntakeControls();
            updateDepositControls();

            // Send telemetry data
            sendTelemetry();

//            autoTransfer();;

            // Perform robot updates
            robot.update();
        }
    }

    // Initializes motor mappings
    private void initializeMotors() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
    }

    // Processes gamepad inputs for movement
    private void handleDriveInput() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;  // Y-axis is inverted
        double rx = gamepad1.right_stick_x;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(y, x), -rx));
        setDrivePower();
    }

    // Calculates and sets motor powers for mecanum drive
    private void setDrivePower() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        updateIntakeControls();
        updateDepositControls();
        sendTelemetry();
    }

    private void updateIntakeControls() {
        if (gamepad2.a) robot.intakeClaw.openClaw();
        if (gamepad2.b) robot.intakeClaw.closeClaw();

        robot.intakeSlide.manualExtension(gamepad2.right_stick_y);

        if (gamepad2.x) robot.intakeV4B.setWristPickPosition();
        if (gamepad2.y) robot.intakeV4B.setWristDropPosition();
        if (gamepad2.right_stick_button) robot.intakeV4B.setWristDefaultPosition();
        if (gamepad2.left_trigger > 0.5) robot.intakeClaw.setOrientationIncrease();
        if (gamepad2.right_trigger > 0.5) robot.intakeClaw.setOrientationDecrease();
    }

    private void updateDepositControls() {
        if (gamepad2.right_bumper) robot.depositClaw.openDepositClaw();
        if (gamepad2.left_bumper) robot.depositClaw.closeDepositClaw();

        robot.depositSlide.manualExtension(gamepad2.left_stick_y);
//
//        if (gamepad1.dpad_up) robot.depositSlide.extendDepositMainSlide();
//        if (gamepad1.dpad_right) robot.depositSlide.retractDepositMainSlide();

        if (gamepad2.dpad_down) robot.depositV4B.setWristPickPosition();
        if (gamepad2.dpad_up) robot.depositV4B.setWristDropPosition();
        if (gamepad2.left_stick_button) robot.depositV4B.setWristSpecimenDropPosition();
    }

//    private void autoTransfer() {
//        if (gamepad1.a) {
//                        robot.depositClaw.openDepositClaw();
//                        sleep(1000);
//                        robot.depositSlide.retractDepositMainSlide();
//                        sleep(1500);
//                        robot.depositClaw.closeDepositClaw();
//                        sleep(1000);
//                        robot.intakeClaw.openClaw();
//                        sleep(1000);
//                        robot.depositSlide.extendDepositMainSlide();
//                        sleep(1500);
//                        robot.depositV4B.setWristDropPosition();
//                        sleep(1000);
//        }
//    }



    private void sendTelemetry() {
        telemetry.addData("Orientation Position", orientationPosition);
        telemetry.addData("Wrist Servo 1 Position", robot.intakeV4B.wristServo1.getPosition());
        telemetry.addData("Wrist Servo 2 Position", robot.intakeV4B.wristServo2.getPosition());
        telemetry.addData("Slide Power", robot.intakeSlide.slideMotor.getPower());
        telemetry.addData("Slide Position", robot.intakeSlide.slideMotor.getCurrentPosition());
        telemetry.update();
    }
}

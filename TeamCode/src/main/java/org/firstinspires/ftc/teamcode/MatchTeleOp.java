package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
            handleDriveInput();
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
        setDrivePower(x, y, rx);
    }

    // Calculates and sets motor powers for mecanum drive
    private void setDrivePower(double x, double y, double rx) {
        double frontLeftPower = y + x + rx;
        double frontRightPower = y - x - rx;
        double backLeftPower = y - x + rx;
        double backRightPower = y + x - rx;

        // Normalize motor powers if any exceeds 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
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
        if (gamepad2.left_bumper) robot.intakeClaw.setOrientationIncrease();
        if (gamepad2.right_bumper) robot.intakeClaw.setOrientationDecrease();
    }

    private void updateDepositControls() {
        if (gamepad2.dpad_right) robot.depositClaw.openDepositClaw();
        if (gamepad2.dpad_left) robot.depositClaw.closeDepositClaw();

        robot.depositSlide.manualExtension(gamepad2.left_stick_y);

        if (gamepad2.dpad_down) robot.depositV4B.setWristPickPosition();
        if (gamepad2.dpad_up) robot.depositV4B.setWristDropPosition();
        if (gamepad2.left_stick_button) robot.depositV4B.setWristSpecimenDropPosition();
    }

    private void sendTelemetry() {
        telemetry.addData("Orientation Position", orientationPosition);
        telemetry.addData("Wrist Servo 1 Position", robot.intakeV4B.wristServo1.getPosition());
        telemetry.addData("Wrist Servo 2 Position", robot.intakeV4B.wristServo2.getPosition());
        telemetry.addData("Slide Power", robot.intakeSlide.slideMotor.getPower());
        telemetry.addData("Slide Position", robot.intakeSlide.slideMotor.getCurrentPosition());
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Servo Teleop", group = "Teleop")
public class ServoTeleOp extends LinearOpMode {
    private Servo iservo1;
    private Servo iservo2;

    private Servo dservo1;
    private Servo dservo2;

    private Servo intakeClawServo;
    private Servo depositClawServo;

    @Override
    public void runOpMode() {

        Robot myBot = new Robot(hardwareMap, telemetry);
        // Initialize the servos
        iservo1 = hardwareMap.get(Servo.class, "wsrv1");
        iservo2 = hardwareMap.get(Servo.class, "wsrv2");

        dservo1 = hardwareMap.get(Servo.class, "dwsrv1");
        dservo2 = hardwareMap.get(Servo.class, "dwsrv2");

        intakeClawServo  = hardwareMap.get(Servo.class, "clsrv");
        depositClawServo  = hardwareMap.get(Servo.class, "dclsrv");

        iservo1.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        iservo1.setPosition(0.5); // Adjust for clockwise
        iservo2.setPosition(0.5); // Adjust for counterclockwise
        dservo1.setPosition(0.5);
        dservo2.setPosition(0.5);

        while (opModeIsActive()) {

            if (gamepad2.x) {
                iservo1.setPosition(0);
                iservo2.setPosition(0);
            }

            if (gamepad2.y) {
                iservo1.setPosition(0.4);
                iservo2.setPosition(0.4);
            }

            intakeClawServo.setPosition(gamepad2.left_stick_y);
            depositClawServo.setPosition(gamepad2.right_stick_y);

            if (gamepad2.dpad_down) {
                dservo1.setPosition(0.27);
                dservo2.setPosition(0.27);
            }

            if (gamepad2.dpad_up) {
                dservo1.setPosition(0);
                dservo2.setPosition(0);
            }

            if (gamepad2.dpad_down) {
                dservo1.setPosition(0.27);
                dservo2.setPosition(0.27);
            }

            if (gamepad2.dpad_up) {
                dservo1.setPosition(1.0);
                dservo2.setPosition(1.0);
            }

            if (gamepad2.left_stick_button) {
                dservo1.setPosition(0);
                dservo2.setPosition(0);
            }


            // Display the servo positions in the telemetry for debugging
            telemetry.addData("Servo1 Position", iservo1.getPosition());
            telemetry.addData("Servo2 Position", iservo2.getPosition());
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);

            telemetry.addData("Deposit Servo1 Position", dservo1.getPosition());
            telemetry.addData("Deposit Servo2 Position", dservo2.getPosition());
            telemetry.addData("Right Stick Y", gamepad2.right_stick_y);

            telemetry.update();
        }
    }
}

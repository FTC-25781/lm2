package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Servo Teleop", group = "Teleop")
public class ServoTeleOp extends LinearOpMode {
    private Servo servo1;
    private Servo servo2;

    @Override
    public void runOpMode() {

        Robot myBot = new Robot(hardwareMap, telemetry);
        // Initialize the servos
        servo1 = hardwareMap.get(Servo.class, "wsrv1");
        servo2 = hardwareMap.get(Servo.class, "wsrv2");

        servo1.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        servo1.setPosition(0.5); // Adjust for clockwise
        servo2.setPosition(0.5); // Adjust for counterclockwise
        while (opModeIsActive()) {
            // Moving forward: servo1 clockwise, servo2 counterclockwise
            servo1.setPosition(gamepad1.left_stick_y*0.5); // Adjust for clockwise
            servo2.setPosition(gamepad1.left_stick_y*0.5); // Adjust for counterclockwise

            // Display the servo positions in the telemetry for debugging
            telemetry.addData("Servo1 Position", servo1.getPosition());
            telemetry.addData("Servo2 Position", servo2.getPosition());
            telemetry.update();
        }
    }
}

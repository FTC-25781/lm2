package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Deposit Servo Teleop", group = "Teleop")
public class DpServoTeleOp extends LinearOpMode {
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;

    @Override
    public void runOpMode() {

        Robot myBot = new Robot(hardwareMap, telemetry);
        // Initialize the servos
        servo1 = hardwareMap.get(Servo.class, "dwsrv1");
        servo2 = hardwareMap.get(Servo.class, "dwsrv2");
        servo3 = hardwareMap.get(Servo.class, "dclsrv");

        servo1.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        servo1.setPosition(0.5); // Adjust for clockwise
        servo2.setPosition(0.5); // Adjust for counterclockwise
        servo3.setPosition(0.5);
        while (opModeIsActive()) {
            // Moving forward: servo1 clockwise, servo2 counterclockwise
            servo1.setPosition(gamepad1.left_stick_y); // Adjust for clockwise
            servo2.setPosition(gamepad1.left_stick_y); // Adjust for counterclockwise
            servo3.setPosition(gamepad1.right_stick_y);


            // Display the servo positions in the telemetry for debugging
            telemetry.addData("Servo1 Position", servo1.getPosition());
            telemetry.addData("Servo2 Position", servo2.getPosition());
            telemetry.addData("Servo3 Position", servo3.getPosition());

            telemetry.update();
        }
    }
}

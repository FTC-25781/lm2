package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeV4BSubsystem {

    public final Servo wristServo1;
    public final Servo wristServo2;

    private static final double INTAKE_POSITION_INCREMENT = 0.01;

    private static final double INTAKE_WRIST_1_DEFAULT = 0.2;
    private static final double INTAKE_WRIST_2_DEFAULT = 0.2;
    private static final double INTAKE_WRIST_1_DROP = 0.4;
    private static final double INTAKE_WRIST_2_DROP = 0.4;
    private static final double INTAKE_WRIST_1_PICKUP = 0;
    private static final double INTAKE_WRIST_2_PICKUP = 0;

    public IntakeV4BSubsystem(HardwareMap hardwareMap) {
        wristServo1 = hardwareMap.get(Servo.class, "wsrv1");
        wristServo2 = hardwareMap.get(Servo.class, "wsrv2");
    }

    public void setWristDropPosition() {
        setWristPosition(INTAKE_WRIST_1_DROP, INTAKE_WRIST_2_DROP);
    }

    public void setWristDefaultPosition() {
        setWristPosition(INTAKE_WRIST_1_DEFAULT, INTAKE_WRIST_2_DEFAULT);
    }

    private void setWristPosition(double pos1, double pos2) {
        wristServo1.setPosition(pos1);
        wristServo2.setPosition(pos2);
    }

    public Action wristPositionAction() {
        return new Action() {
            private double currentPos1 = wristServo1.getPosition();
            private double currentPos2 = wristServo2.getPosition();

            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                currentPos1 = adjustPosition(currentPos1, INTAKE_WRIST_1_PICKUP);
                currentPos2 = adjustPosition(currentPos2, INTAKE_WRIST_2_PICKUP);

                wristServo1.setPosition(currentPos1);
                wristServo2.setPosition(currentPos2);

                return !(isAtTarget(currentPos1, INTAKE_WRIST_1_PICKUP) && isAtTarget(currentPos2, INTAKE_WRIST_2_PICKUP));
            }

            private double adjustPosition(double current, double target) {
                return current < target
                        ? Math.min(current + INTAKE_POSITION_INCREMENT, target)
                        : Math.max(current - INTAKE_POSITION_INCREMENT, target);
            }

            private boolean isAtTarget(double current, double target) {
                return Math.abs(current - target) <= INTAKE_POSITION_INCREMENT;
            }
        };
    }

    public void update() {

    }
}

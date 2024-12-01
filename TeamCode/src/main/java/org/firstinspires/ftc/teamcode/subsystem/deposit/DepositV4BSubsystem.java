package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

public class DepositV4BSubsystem implements Subsystem {

    private final Servo wristServo1;
    private final Servo wristServo2;

    private static final double DEPOSIT_WRIST_1_SPECIMEN_DROP = 0;
    private static final double DEPOSIT_WRIST_2_SPECIMEN_DROP = 0;
    private static final double DEPOSIT_WRIST_1_DROP = 1.0;
    private static final double DEPOSIT_WRIST_2_DROP = 1.0;
    private static final double DEPOSIT_WRIST_1_PICKUP = 0.27;
    private static final double DEPOSIT_WRIST_2_PICKUP = 0.27;

    public DepositV4BSubsystem(HardwareMap hardwareMap) {
        wristServo1 = hardwareMap.get(Servo.class, "dwsrv1");
        wristServo2 = hardwareMap.get(Servo.class, "dwsrv2");

        wristServo2.setDirection(Servo.Direction.REVERSE);
    }

    public void setWristDropPosition() {
        setWristPosition(DEPOSIT_WRIST_1_DROP, DEPOSIT_WRIST_2_DROP);
    }

    public void setWristPickPosition() {
        setWristPosition(DEPOSIT_WRIST_1_PICKUP, DEPOSIT_WRIST_2_PICKUP);
    }

    private void setWristPosition(double pos1, double pos2) {
        wristServo1.setPosition(pos1);
        wristServo2.setPosition(pos2);
    }

    @Override
    public void update() {
        // No-op: Implement update logic if necessary.
    }
}

package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

public class DepositClawSubsystem implements Subsystem {

    private final Servo clawServo;

    private static final double CLAW_OPEN_POS = 0.3;
    private static final double CLAW_CLOSED_POS = 0.55;

    public DepositClawSubsystem(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "dclsrv");
    }

    public void runToPreset() {
        closeDepositClaw();
    }

    public void openDepositClaw() {
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    public void closeDepositClaw() {
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    @Override
    public void update() {
        // No-op: Implement any periodic updates if needed.
    }
}

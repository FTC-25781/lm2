package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DepositSlideSubsystem {

    private final DigitalChannel depositLimitSwitch;
    private final DcMotor verticalSlideMotor;
    private final DcMotor verticalSlideMotor2;

    private static final int SLIDE_EXTEND_POS = 800;
    private static final int SLIDE_RETRACT_POS = 0;
    private static final double SLIDE_EXTEND_SPEED = 0.5;
    private static final double MIN_SPEED = 0.1;

    public DepositSlideSubsystem(HardwareMap hardwareMap) {
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "vsmot");
        verticalSlideMotor2 = hardwareMap.get(DcMotor.class, "vsmot2");
        depositLimitSwitch = hardwareMap.get(DigitalChannel.class, "dpltsw");

        depositLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalSlideMotor2.setDirection(DcMotor.Direction.REVERSE);  // Ensuring synchronized opposite movement
        resetMotors();
    }

    // Reset encoders and stop movement.
    private void resetMotors() {
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void manualExtension(double power) {
        if ((power > 0 && depositLimitSwitch.getState()) || (power < 0 && verticalSlideMotor.getCurrentPosition() > SLIDE_RETRACT_POS)) {
            verticalSlideMotor.setPower(clampPower(power));
            verticalSlideMotor2.setPower(clampPower(power));
        } else {
            stopMotors();
        }
    }

    public void extendDepositMainSlide() {
        if (depositLimitSwitch.getState()) {
            int currentPosition = verticalSlideMotor.getCurrentPosition();
            int distanceToTarget = SLIDE_EXTEND_POS - currentPosition;

            double proportionalSpeed = Math.max(MIN_SPEED, (double) distanceToTarget / 1000);
            double power = proportionalSpeed * SLIDE_EXTEND_SPEED;
            verticalSlideMotor.setPower(distanceToTarget > 0 ? power : 0);
            verticalSlideMotor2.setPower(distanceToTarget > 0 ? power : 0);
        } else {
            stopMotors();
        }
    }

    public void retractDepositMainSlide() {
        if (!depositLimitSwitch.getState() || verticalSlideMotor.getCurrentPosition() <= SLIDE_RETRACT_POS) {
            stopMotors();
            resetMotors();
        } else {
            verticalSlideMotor.setPower(-SLIDE_EXTEND_SPEED);
            verticalSlideMotor2.setPower(-SLIDE_EXTEND_SPEED);
        }
    }

    private void stopMotors() {
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private double clampPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }

    public boolean isLimitSwitchPressed() {
        return !depositLimitSwitch.getState();
    }

    public void update() {

    }
}

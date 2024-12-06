package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DepositSlideSubsystem {

    private final DigitalChannel depositLimitSwitch;
    private final DcMotor verticalSlideMotor;
    private final DcMotor verticalSlideMotor2;

    private static final int SLIDE_EXTEND_POS = 10000;
    private static final int SLIDE_RETRACT_POS = 500;
    private static final double SLIDE_EXTEND_SPEED = 0.5;
    private static final double MIN_SPEED = 0.1;

    public DepositSlideSubsystem(HardwareMap hardwareMap) {
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "vsmot");
        verticalSlideMotor2 = hardwareMap.get(DcMotor.class, "vsmot2");
        depositLimitSwitch = hardwareMap.get(DigitalChannel.class, "dpltsw");


        verticalSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        verticalSlideMotor2.setDirection(DcMotor.Direction.FORWARD);

        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        depositLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void manualExtension(double power) {
        if (!depositLimitSwitch.getState() && power > 0) {
            // Block further extension when limit switch is pressed
            verticalSlideMotor.setPower(0);
            verticalSlideMotor2.setPower(0);
        } else {
            verticalSlideMotor.setPower(clampPower(power*-1));
            verticalSlideMotor2.setPower(clampPower(power*-1));
        }
    }

    public void extendDepositMainSlide() {
        int currentPosition = verticalSlideMotor.getCurrentPosition();
        int distanceToTarget = SLIDE_EXTEND_POS - currentPosition;

        verticalSlideMotor.setTargetPosition(SLIDE_EXTEND_POS);
        verticalSlideMotor2.setTargetPosition(SLIDE_EXTEND_POS);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);
        verticalSlideMotor2.setPower(1);

        while (verticalSlideMotor.isBusy() && verticalSlideMotor2.isBusy()) {
            double proportionalSpeed = Math.max(MIN_SPEED, (double) distanceToTarget / 1000);
            verticalSlideMotor.setPower(distanceToTarget > 0 ? proportionalSpeed * SLIDE_EXTEND_SPEED : 0);
            verticalSlideMotor2.setPower(distanceToTarget > 0 ? proportionalSpeed * SLIDE_EXTEND_SPEED : 0);
        }

        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void retractDepositMainSlide() {
        int currentPosition = verticalSlideMotor.getCurrentPosition();
        int distanceToTarget = currentPosition - SLIDE_RETRACT_POS;

        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (verticalSlideMotor.isBusy() && verticalSlideMotor2.isBusy() && depositLimitSwitch.getState()) {
            currentPosition = verticalSlideMotor.getCurrentPosition();
            distanceToTarget = currentPosition - SLIDE_RETRACT_POS;

            double proportionalSpeed = Math.max(MIN_SPEED, (double) distanceToTarget / 1000);
            verticalSlideMotor.setPower(-proportionalSpeed * SLIDE_EXTEND_SPEED);
            verticalSlideMotor2.setPower(-proportionalSpeed * SLIDE_EXTEND_SPEED);
        }

        verticalSlideMotor.setPower(0);
        verticalSlideMotor2.setPower(0);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void stopSlides() {
        verticalSlideMotor.setPower(0);
        verticalSlideMotor2.setPower(0);
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

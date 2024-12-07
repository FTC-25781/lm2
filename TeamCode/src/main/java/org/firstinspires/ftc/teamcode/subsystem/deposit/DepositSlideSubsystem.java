package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.UltrasonicDistanceSensor;
import org.firstinspires.ftc.teamcode.subsystem.deposit.DepositV4BSubsystem;

import java.util.concurrent.TimeUnit;

public class DepositSlideSubsystem {

    private final DigitalChannel depositLimitSwitch;
    public final DcMotor verticalSlideMotor;
    public final DcMotor verticalSlideMotor2;
    public UltrasonicDistanceSensor rangeSensor;
    public DepositV4BSubsystem depositV4B;

    private static final int SLIDE_EXTEND_POS = 10000;
    private static final int SLIDE_RETRACT_POS = 1000;
    private static final double SLIDE_EXTEND_SPEED = 0.5;
    private static final double MIN_SPEED = 0.1;
    public final Telemetry telemetry;

    public DepositSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        depositV4B = new DepositV4BSubsystem(hardwareMap);

        verticalSlideMotor = hardwareMap.get(DcMotor.class, "vsmot");
        verticalSlideMotor2 = hardwareMap.get(DcMotor.class, "vsmot2");
        depositLimitSwitch = hardwareMap.get(DigitalChannel.class, "dpltsw");
        rangeSensor = new UltrasonicDistanceSensor(hardwareMap.get(AnalogInput.class, "vdist1"));


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
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        verticalSlideMotor.setPower(0.8);
        verticalSlideMotor2.setPower(0.8);

        while (rangeSensor.getDistance(DistanceUnit.INCH) < 42) {
            telemetry.addData("Distance: ", rangeSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            if (timer.time(TimeUnit.MILLISECONDS) > 1500) {
                depositV4B.setWristSpecimenDropPosition();
            }
        }
        verticalSlideMotor.setPower(0);
        verticalSlideMotor2.setPower(0);
    }

    public void retractDepositMainSlide() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        verticalSlideMotor.setPower(-0.8);
        verticalSlideMotor2.setPower(-0.8);

        while (rangeSensor.getDistance(DistanceUnit.INCH) > 16.50) {
            telemetry.addData("Distance: ", rangeSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        verticalSlideMotor.setPower(0);
        verticalSlideMotor2.setPower(0);
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

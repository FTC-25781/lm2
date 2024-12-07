package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.deposit.DepositClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.deposit.DepositSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.deposit.DepositV4BSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeV4BSubsystem;

public class Robot {

    public final IntakeV4BSubsystem intakeV4B;
    public final IntakeSlideSubsystem intakeSlide;
    public final IntakeClawSubsystem intakeClaw;
    public final DepositV4BSubsystem depositV4B;
    public final DepositSlideSubsystem depositSlide;
    public final DepositClawSubsystem depositClaw;
    public final MecanumDrive drive;
    public final Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeV4B = new IntakeV4BSubsystem(hardwareMap);
        intakeSlide = new IntakeSlideSubsystem(hardwareMap, telemetry);
        intakeClaw = new IntakeClawSubsystem(hardwareMap);
        depositV4B = new DepositV4BSubsystem(hardwareMap);
        depositSlide = new DepositSlideSubsystem(hardwareMap, telemetry);
        depositClaw = new DepositClawSubsystem(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public void preset() {
        depositClaw.runToPreset();
        intakeClaw.runToPreset();
    }

    public void startIntakePickup() {
        intakeSlide.extendMainSlide();
        intakeClaw.openClaw();
        intakeV4B.setWristDefaultPosition();
    }

    public void startIntakeDrop() {
        intakeV4B.setWristPickPosition();
        intakeClaw.closeClaw();
        intakeV4B.setWristDropPosition();
        intakeSlide.retractMainSlide();
    }
//    public Action startIntakeeDrop() {
//        new SequentialAction(
//          intakeV4B.wristPositionAction()
//        );
//    }

    public void startDepositPickup() {
        intakeClaw.openClaw();
        intakeV4B.setWristDefaultPosition();
        depositClaw.openDepositClaw();
        depositV4B.setWristPickPosition();
        depositSlide.retractDepositMainSlide();
        depositClaw.closeDepositClaw();
    }

    public void startDepositDrop() {
        depositSlide.extendDepositMainSlide();
        depositV4B.setWristDropPosition();
        depositClaw.openDepositClaw();
        depositClaw.closeDepositClaw();
    }

    public void update() {
        intakeClaw.update();
        intakeSlide.update();
        intakeV4B.update();
        depositClaw.update();
        depositSlide.update();
        depositV4B.update();
        drive.updatePoseEstimate();
    }
}

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Blue/Red Active Auto", group = "")
public class BlueRedActiveAuto extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        drive = new MecanumDrive(hardwareMap, new Pose2d(62.8, 20.5, 0));

        robot = new Robot(hardwareMap, telemetry);

        runtime.reset();
        robot.depositSlide.stopSlides();

        waitForStart();

        Action bluered1 = drive.actionBuilder(new Pose2d(62.8,20.5,0))
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //first time going under basket
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) { //dep slides to drop pos
                        robot.depositSlide.extendDepositMainSlide();
                        robot.depositV4B.setWristDropPosition();
                        robot.depositClaw.openDepositClaw();
                        robot.intakeV4B.setWristDefaultPosition();
                        robot.depositV4B.setWristPickPosition();
                        robot.depositSlide.retractDepositMainSlide();
                        return true;
                }})
                .strafeToLinearHeading(new Vector2d(85.5,47.95), 0) //first sample
                .stopAndAdd(new Action() { //intake slides go out
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.intakeSlide.extendMainSlide();
                        robot.intakeClaw.openClaw();
                        robot.intakeClaw.setOrientationIncrease();
                        robot.intakeV4B.setWristPickPosition();
                        robot.intakeClaw.closeClaw();
                        robot.intakeV4B.setWristDropPosition();
                        robot.depositClaw.openDepositClaw();
                        robot.intakeClaw.setOrientationIncrease();
                        return true;
                }})
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //going back under basket
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) { //dep slides to drop pos
                        robot.depositSlide.extendDepositMainSlide();
                        robot.depositV4B.setWristDropPosition();
                        robot.depositClaw.openDepositClaw();
                        robot.intakeV4B.setWristDefaultPosition();
                        robot.depositV4B.setWristPickPosition();
                        robot.depositSlide.retractDepositMainSlide();
                        return true;
                    }})
                .strafeToLinearHeading(new Vector2d(85.5, 58.89), 0) //second sample
                .stopAndAdd(new Action() { //intake slides go out
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.intakeSlide.extendMainSlide();
                        robot.intakeClaw.openClaw();
                        robot.intakeClaw.setOrientationIncrease();
                        robot.intakeV4B.setWristPickPosition();
                        robot.intakeClaw.closeClaw();
                        robot.intakeV4B.setWristDropPosition();
                        robot.depositClaw.openDepositClaw();
                        robot.intakeClaw.setOrientationIncrease();
                        return true;
                    }})
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //going back under basket
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) { //dep slides to drop pos
                        robot.depositSlide.extendDepositMainSlide();
                        robot.depositV4B.setWristDropPosition();
                        robot.depositClaw.openDepositClaw();
                        robot.intakeV4B.setWristDefaultPosition();
                        robot.depositV4B.setWristPickPosition();
                        robot.depositSlide.retractDepositMainSlide();
                        return true;
                    }})
                .strafeToLinearHeading(new Vector2d(95.86, 48.91), -80) //turn sideways and third sample
                .stopAndAdd(new Action() { //intake slides go out
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.intakeSlide.extendMainSlide();
                        robot.intakeClaw.openClaw();
                        robot.intakeClaw.setOrientationIncrease();
                        robot.intakeV4B.setWristPickPosition();
                        robot.intakeClaw.closeClaw();
                        robot.intakeV4B.setWristDropPosition();
                        robot.depositClaw.openDepositClaw();
                        robot.intakeClaw.setOrientationIncrease();
                        return true;
                    }})
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //going back under basket
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) { //dep slides to drop pos
                        robot.depositSlide.extendDepositMainSlide();
                        robot.depositV4B.setWristDropPosition();
                        robot.depositClaw.openDepositClaw();
                        robot.intakeV4B.setWristDefaultPosition();
                        robot.depositV4B.setWristPickPosition();
                        robot.depositSlide.retractDepositMainSlide();
                        return true;
                    }})
                .strafeToLinearHeading(new Vector2d(115.52, 60.20), 0)//going to park part 1
                .strafeToLinearHeading(new Vector2d(122.77, 26.01), 0)//going to park part 2
                .build();

        Actions.runBlocking(new SequentialAction(bluered1));

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
    }
}

package org.firstinspires.ftc.teamcode;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.deposit.DepositClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.deposit.DepositV4BSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.deposit.DepositSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeV4BSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeSlideSubsystem;
import org.firstinspires.ftc.teamcode.blockDetection;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "Blue/Red Active Auto", group = "")
public class BlueRedActiveAuto extends LinearOpMode {

    MecanumDrive drive;
    Robot robot;
    //intake hardware
    public DcMotor hsmot;
    public Servo clsrv;
    public Servo orsrv;
    public Servo wsrv1;
    public Servo wsrv2;
    public DigitalChannel inltsw;
    //deposit hardware
    public DcMotor vsmot;
    public DcMotor vsmot2;
    public Servo dclsrv;
    public Servo dwsrv1;
    public Servo dwsrv2;
    public DigitalChannel dpltsw;

    ElapsedTime timer = new ElapsedTime();



    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        drive = new MecanumDrive(hardwareMap, new Pose2d(62.8, 20.5, 0));
//        robot = new Robot(hardwareMap, telemetry);


        runtime.reset();
        waitForStart();

        Action bluered1 = drive.actionBuilder(new Pose2d(62.8,20.5,0))
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //first time going under basket
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) { //dep slides go up a bit

                        vsmot.setTargetPosition(10);
                        vsmot2.setTargetPosition(10);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(1);
                        vsmot2.setPower(1);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //clsrv opens so deposit can come down
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        clsrv.setPosition(0.69);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist to default so deposit can come down
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        wsrv1.setPosition(0.4);
                        wsrv2.setPosition(0.4);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep clsrv opens
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        dclsrv.setPosition(0.42);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep wrist pick pos
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        dwsrv1.setPosition(0.57);
                        dwsrv2.setPosition(0.45);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep slides go all the way down so claw can pick sample
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        vsmot.setTargetPosition(0);
                        vsmot2.setTargetPosition(0);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(1);
                        vsmot2.setPower(1);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //dep claw close
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        dclsrv.setPosition(0.52);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep slides go up to buckets
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        vsmot.setTargetPosition(100);
                        vsmot2.setTargetPosition(100);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(1);
                        vsmot2.setPower(1);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //dep wrist set to drop pos
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        dwsrv1.setPosition(0.15);
                        dwsrv2.setPosition(0.1);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep claw opens
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        dclsrv.setPosition(0.42);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep wrist set to default
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        dwsrv1.setPosition(1.0);
                        dwsrv2.setPosition(1.0);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep lide come back down
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        vsmot.setTargetPosition(0);
                        vsmot2.setTargetPosition(0);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(1);
                        vsmot2.setPower(1);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .strafeToLinearHeading(new Vector2d(82,47.95), 0) //first sample
                .stopAndAdd(new Action() { //intake slides go out
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        hsmot.setTargetPosition(100);
                        hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hsmot.setPower(1);

                        if (hsmot.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //intake claw opens
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        clsrv.setPosition(0.69);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist goes to pick pos
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        wsrv1.setPosition(0.55);
                        wsrv2.setPosition(0.55);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw closes
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        clsrv.setPosition(0.9);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist to drop pos
                    ElapsedTime timer = new ElapsedTime();

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        wsrv1.setPosition(0.25);
                        wsrv2.setPosition(0.25);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake slides go back in
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        hsmot.setTargetPosition(0);
                        hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hsmot.setPower(1);

                        if (hsmot.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //going back under basket
                .strafeToLinearHeading(new Vector2d(82.62, 58.89), 0) //second sample
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //going back under basket
                .strafeToLinearHeading(new Vector2d(95.86, 48.91), -80) //turn sideways and third sample
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //going back under basket
                .strafeToLinearHeading(new Vector2d(115.52, 60.20), 0)
                .strafeToLinearHeading(new Vector2d(122.77, 26.01), 0)
                .build();
//                .stopAndAdd(new Action() {
//                    ElapsedTime timer = new ElapsedTime();

//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
////                        clsrv.setPosition(1);
//
//                        if (timer.time(TimeUnit.SECONDS) < 4)
//                            return true;
//                        else
//                            return false;
//                    }
//                })
//                .stopAndAdd((p) -> {
//                    ElapsedTime timer = new ElapsedTime();
//
//                    clsrv.setPosition(1);
//
//                    if (timer.now(TimeUnit.SECONDS) < 1)
//                        return true;
//                    else
//                        return false;
//                })
//                .stopAndAdd(new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//                        hsmot.setTargetPosition(100);
//                        hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        hsmot.setPower(1);
//
//                        if (hsmot.isBusy()) {
//                            return true;
//                        } else {
//                            return false;
//                        }
//                    }
//                })
//



        Actions.runBlocking(new SequentialAction(

                bluered1

        ));

        while (opModeIsActive()) {

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
    }
}

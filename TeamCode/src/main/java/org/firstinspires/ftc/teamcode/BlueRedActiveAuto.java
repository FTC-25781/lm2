package org.firstinspires.ftc.teamcode;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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

    public static final int HSMOT_PICKUP = 1000;
    public static final int HSMOT_BACK = 500;
    public static final double INTAKE_CLAW_OPEN = 0.77;
    public static final double INTAKE_CLAW_CLOSE = 1.0;
    public static final double SAMPLE_CLAW_ORIENT = 0;
    public static final double INTAKE_TO_DEPOSIT_CLAW_ORIENT = 0.55;
    public static final double INTAKE_WRIST1_DROP = 0.38;
    public static final double INTAKE_WRIST2_DROP = 0.38;
    public static final double INTAKE_WRIST1_PICK = 0.05;
    public static final double INTAKE_WRIST2_PICK = 0.05;
    public static final double INTAKE_WRIST1_DEFAULT = 0.2;
    public static final double INTAKE_WRIST2_DEFAULT = 0.2;
    public static final int VSMOT_DOWN = 4000;
    public static final int VSMOT_UP = 10000;
    public static final double DEPOSIT_CLAW_OPEN = 0.35;
    public static final double DEPOSIT_CLAW_CLOSE = 0.55;
    public static final double DEPOSIT_WRIST1_DROP = 1.0;
    public static final double DEPOSIT_WRIST2_DROP = 1.0;
    public static final double DEPOSIT_WRIST1_PICK = 0.27;
    public static final double DEPOSIT_WRIST2_PICK = 0.27;
    public static final double DEPOSIT_WRIST1_DEFAULT = 0;
    public static final double DEPOSIT_WRIST2_DEFAULT = 0;
    public static final double DEPOSIT_MOTOR_POWER = 0.6;
    public static final double INTAKE_MOTOR_POWER = 0.6;


    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        drive = new MecanumDrive(hardwareMap, new Pose2d(62.8, 20.5, 0));
        hsmot  = hardwareMap.get(DcMotor.class, "hsmot");
        vsmot = hardwareMap.get(DcMotor.class, "vsmot");
        vsmot2 = hardwareMap.get(DcMotor.class, "vsmot2");

        vsmot.setDirection(DcMotor.Direction.REVERSE);
        vsmot2.setDirection(DcMotor.Direction.FORWARD);
        hsmot.setDirection(DcMotor.Direction.FORWARD);

        hsmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vsmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vsmot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wsrv1 = hardwareMap.get(Servo.class, "wsrv1");
        wsrv2 = hardwareMap.get(Servo.class, "wsrv2");

        dwsrv1 = hardwareMap.get(Servo.class, "dwsrv1");
        dwsrv2 = hardwareMap.get(Servo.class, "dwsrv2");

        clsrv  = hardwareMap.get(Servo.class, "clsrv");
        dclsrv  = hardwareMap.get(Servo.class, "dclsrv");
        orsrv =  hardwareMap.get(Servo.class, "orsrv");

        dpltsw = hardwareMap.get(DigitalChannel.class, "dpltsw");
        inltsw = hardwareMap.get(DigitalChannel.class, "inltsw");


        wsrv1.setDirection(Servo.Direction.REVERSE);


        robot = new Robot(hardwareMap, telemetry);


        runtime.reset();
        waitForStart();

        Action bluered1 = drive.actionBuilder(new Pose2d(62.8,20.5,0))
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //first time going under basket
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) { //dep slides to drop pos

                        vsmot.setTargetPosition(VSMOT_UP);
                        vsmot2.setTargetPosition(VSMOT_UP);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(DEPOSIT_MOTOR_POWER);
                        vsmot2.setPower(DEPOSIT_MOTOR_POWER);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //dep arm set to drop pos
                    ElapsedTime timer = new ElapsedTime();

                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dwsrv1.setPosition(DEPOSIT_WRIST1_DROP);
                        dwsrv2.setPosition(DEPOSIT_WRIST2_DROP);

                        if (timer.time(TimeUnit.SECONDS) < 2)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist goes to default pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        wsrv1.setPosition(INTAKE_WRIST1_DEFAULT);
                        wsrv2.setPosition(INTAKE_WRIST2_DEFAULT);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep wrist set to pick
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dwsrv1.setPosition(DEPOSIT_WRIST1_PICK);
                        dwsrv2.setPosition(DEPOSIT_WRIST2_PICK);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep slides come back down
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        vsmot.setTargetPosition(VSMOT_DOWN);
                        vsmot2.setTargetPosition(VSMOT_DOWN);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(DEPOSIT_MOTOR_POWER);
                        vsmot2.setPower(DEPOSIT_MOTOR_POWER);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .strafeToLinearHeading(new Vector2d(85.5,47.95), 0) //first sample
                .stopAndAdd(new Action() { //intake slides go out
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        hsmot.setTargetPosition(HSMOT_PICKUP);
                        hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hsmot.setPower(INTAKE_MOTOR_POWER);

                        if (hsmot.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //intake claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        clsrv.setPosition(INTAKE_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw orients
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        orsrv.setPosition(SAMPLE_CLAW_ORIENT);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist goes to pick pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        wsrv1.setPosition(INTAKE_WRIST1_PICK);
                        wsrv2.setPosition(INTAKE_WRIST2_PICK);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw closes
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        clsrv.setPosition(INTAKE_CLAW_CLOSE);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist to drop pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        wsrv1.setPosition(INTAKE_WRIST1_DROP);
                        wsrv2.setPosition(INTAKE_WRIST2_DROP);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep clsrv opens for clearance for block
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //orient servo to deposit servo orient
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        orsrv.setPosition(INTAKE_TO_DEPOSIT_CLAW_ORIENT);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake slides go back in
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        hsmot.setTargetPosition(HSMOT_BACK);
                        hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hsmot.setPower(INTAKE_MOTOR_POWER);

                        if (hsmot.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //going back under basket
                .stopAndAdd(new Action() { //dep claw close
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_CLOSE);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        clsrv.setPosition(INTAKE_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) { //dep slides to drop pos

                        vsmot.setTargetPosition(VSMOT_UP);
                        vsmot2.setTargetPosition(VSMOT_UP);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(DEPOSIT_MOTOR_POWER);
                        vsmot2.setPower(DEPOSIT_MOTOR_POWER);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //dep arm set to drop pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dwsrv1.setPosition(DEPOSIT_WRIST1_DROP);
                        dwsrv2.setPosition(DEPOSIT_WRIST2_DROP);

                        if (timer.time(TimeUnit.SECONDS) < 5)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep wrist set to pick
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dwsrv1.setPosition(DEPOSIT_WRIST1_PICK);
                        dwsrv2.setPosition(DEPOSIT_WRIST2_PICK);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep slides come back down
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        vsmot.setTargetPosition(VSMOT_DOWN);
                        vsmot2.setTargetPosition(VSMOT_DOWN);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(DEPOSIT_MOTOR_POWER);
                        vsmot2.setPower(DEPOSIT_MOTOR_POWER);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .strafeToLinearHeading(new Vector2d(85.5, 58.89), 0) //second sample
                .stopAndAdd(new Action() { //intake slides go out
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        hsmot.setTargetPosition(HSMOT_PICKUP);
                        hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hsmot.setPower(INTAKE_MOTOR_POWER);

                        if (hsmot.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //intake claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        clsrv.setPosition(INTAKE_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw orients
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        orsrv.setPosition(SAMPLE_CLAW_ORIENT);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist goes to pick pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        wsrv1.setPosition(INTAKE_WRIST1_PICK);
                        wsrv2.setPosition(INTAKE_WRIST2_PICK);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw closes
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        clsrv.setPosition(INTAKE_CLAW_CLOSE);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist to drop pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        wsrv1.setPosition(INTAKE_WRIST1_DROP);
                        wsrv2.setPosition(INTAKE_WRIST2_DROP);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep clsrv opens for clearance for block
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //orient servo to deposit servo orient
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        orsrv.setPosition(INTAKE_TO_DEPOSIT_CLAW_ORIENT);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake slides go back in
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        hsmot.setTargetPosition(HSMOT_BACK);
                        hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hsmot.setPower(INTAKE_MOTOR_POWER);

                        if (hsmot.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //going back under basket
                .stopAndAdd(new Action() { //dep claw close
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_CLOSE);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        clsrv.setPosition(INTAKE_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) { //dep slides to drop pos

                        vsmot.setTargetPosition(VSMOT_UP);
                        vsmot2.setTargetPosition(VSMOT_UP);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(DEPOSIT_MOTOR_POWER);
                        vsmot2.setPower(DEPOSIT_MOTOR_POWER);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //dep arm set to drop pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dwsrv1.setPosition(DEPOSIT_WRIST1_DROP);
                        dwsrv2.setPosition(DEPOSIT_WRIST2_DROP);

                        if (timer.time(TimeUnit.SECONDS) < 5)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep wrist set to pick
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dwsrv1.setPosition(DEPOSIT_WRIST1_PICK);
                        dwsrv2.setPosition(DEPOSIT_WRIST2_PICK);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep slides come back down
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        vsmot.setTargetPosition(VSMOT_DOWN);
                        vsmot2.setTargetPosition(VSMOT_DOWN);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(DEPOSIT_MOTOR_POWER);
                        vsmot2.setPower(DEPOSIT_MOTOR_POWER);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .strafeToLinearHeading(new Vector2d(95.86, 48.91), -80) //turn sideways and third sample
                .stopAndAdd(new Action() { //intake slides go out
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        hsmot.setTargetPosition(HSMOT_PICKUP);
                        hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hsmot.setPower(INTAKE_MOTOR_POWER);

                        if (hsmot.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //intake claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        clsrv.setPosition(INTAKE_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw orients
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        orsrv.setPosition(SAMPLE_CLAW_ORIENT);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist goes to pick pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        wsrv1.setPosition(INTAKE_WRIST1_PICK);
                        wsrv2.setPosition(INTAKE_WRIST2_PICK);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw closes
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        clsrv.setPosition(INTAKE_CLAW_CLOSE);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake wrist to drop pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        wsrv1.setPosition(INTAKE_WRIST1_DROP);
                        wsrv2.setPosition(INTAKE_WRIST2_DROP);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep clsrv opens for clearance for block
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //orient servo to deposit servo orient
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        orsrv.setPosition(INTAKE_TO_DEPOSIT_CLAW_ORIENT);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake slides go back in
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        hsmot.setTargetPosition(HSMOT_BACK);
                        hsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hsmot.setPower(INTAKE_MOTOR_POWER);

                        if (hsmot.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), -45) //going back under basket
                .stopAndAdd(new Action() { //dep claw close
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_CLOSE);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //intake claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        clsrv.setPosition(INTAKE_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) { //dep slides to drop pos

                        vsmot.setTargetPosition(VSMOT_UP);
                        vsmot2.setTargetPosition(VSMOT_UP);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(DEPOSIT_MOTOR_POWER);
                        vsmot2.setPower(DEPOSIT_MOTOR_POWER);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .stopAndAdd(new Action() { //dep arm set to drop pos
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dwsrv1.setPosition(DEPOSIT_WRIST1_DROP);
                        dwsrv2.setPosition(DEPOSIT_WRIST2_DROP);

                        if (timer.time(TimeUnit.SECONDS) < 5)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep claw opens
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dclsrv.setPosition(DEPOSIT_CLAW_OPEN);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep wrist set to pick
                    ElapsedTime timer = new ElapsedTime();
                    private boolean initialized = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        if (!initialized) {
                            timer.reset();
                            initialized = true;
                        }

                        dwsrv1.setPosition(DEPOSIT_WRIST1_PICK);
                        dwsrv2.setPosition(DEPOSIT_WRIST2_PICK);

                        if (timer.time(TimeUnit.SECONDS) < 1)
                            return true;
                        else
                            return false;
                    }
                })
                .stopAndAdd(new Action() { //dep slides come back down
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                        vsmot.setTargetPosition(VSMOT_DOWN);
                        vsmot2.setTargetPosition(VSMOT_DOWN);
                        vsmot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        vsmot.setPower(DEPOSIT_MOTOR_POWER);
                        vsmot2.setPower(DEPOSIT_MOTOR_POWER);

                        if (vsmot.isBusy() && vsmot2.isBusy()) {
                            return true;
                        } else {
                            return false;
                        }
                    }
                })
                .strafeToLinearHeading(new Vector2d(115.52, 60.20), 0)//going to park part 1
                .strafeToLinearHeading(new Vector2d(122.77, 26.01), 0)//going to park part 2
                .build();



//        Actions.runBlocking(new SequentialAction(
//                bluered1
//        ));
        if(!inltsw.getState()) {
            hsmot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hsmot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(!dpltsw.getState()) {
            vsmot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vsmot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            vsmot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vsmot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(bluered1),
                        telemetryPacket -> {
                            telemetry.addData("vsmot position", vsmot.getCurrentPosition());
                            telemetry.addData("vsmot2 position", vsmot2.getCurrentPosition());
                            telemetry.update();

                            // repeadtely running code
                            return opModeIsActive();
                        }
            ));

        while (opModeIsActive()) {
//            if(!inltsw.getState()) {
//                hsmot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                hsmot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//
//            if(!dpltsw.getState()) {
//                vsmot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                vsmot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                vsmot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                vsmot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//
//            telemetry.addData("vsmot position", vsmot.getCurrentPosition());
//            telemetry.addData("vsmot2 position", vsmot2.getCurrentPosition());
//
//            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
    }
}

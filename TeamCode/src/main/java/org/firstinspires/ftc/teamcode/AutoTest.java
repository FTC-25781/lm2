package org.firstinspires.ftc.teamcode;



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
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;



@Autonomous(name = "Auto Test", group = "")
public class AutoTest extends LinearOpMode {

    MecanumDrive drive;
    public DcMotor slideMotor;
    public ServoImplEx flipLeftIntake;
    public ServoImplEx flipRightIntake;


    public CRServoImplEx intakeMotor;


    public ServoImplEx flipRightDeposit;
    public ServoImplEx flipLeftDeposit;
    public ServoImplEx pitch;
    public ServoImplEx claw;

    public DcMotorImplEx slidesRight;
    public DcMotorImplEx slidesLeft;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        drive = new MecanumDrive(hardwareMap, new Pose2d(62.8, 20.5, 0));


        runtime.reset();
        waitForStart();

        Action traj1 = drive.actionBuilder(new Pose2d(62.8,20.5,0))
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), 45)
//                .stopAndAdd()
                .strafeToLinearHeading(new Vector2d(82,47.95), 0)
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), 45)
                .strafeToLinearHeading(new Vector2d(82.62, 58.89), 0)
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), 45)
                .strafeToLinearHeading(new Vector2d(98.86, 48.91), 90)
                .strafeToLinearHeading(new Vector2d(72.0, 54.7), 45)
                .build();

//        Action traj2 = drive.actionBuilder(new Pose2d(0,26, 45))
//                .strafeToLinearHeading(new Vector2d(10,26), 0)
//                .build();
////
//      Action traj3 = drive.actionBuilder(new Pose2d(10, 26, 0))
//              .strafeToLinearHeading(new Vector2d(0,26), 45)
//                .build();
////
//        Action traj4 = drive.actionBuilder(new Pose2d(10, 26, 45))
//                .strafeToLinearHeading(new Vector2d(20,33), 0)
//                .build();
////
//        Action traj5 = drive.actionBuilder(new Pose2d(10, 26, 45))
//                .strafeToLinearHeading(new Vector2d(0,26), 45)
//                .build();
//
//        Action traj6 = drive.actionBuilder(new Pose2d(10, 26, 45))
//                .strafeToLinearHeading(new Vector2d(20,33), 90)
//                .build();

        Actions.runBlocking(new SequentialAction(

                traj1

        ));

        while (opModeIsActive()) {



            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
    }
}
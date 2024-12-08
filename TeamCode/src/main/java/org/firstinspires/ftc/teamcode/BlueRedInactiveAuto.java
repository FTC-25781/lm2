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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;



@Autonomous(name = "Blue/Red Inactive Auto", group = "")
public class BlueRedInactiveAuto extends LinearOpMode {

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
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {


        // MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        drive = new MecanumDrive(hardwareMap, new Pose2d(63.32, 19.15, 0));
//        robot = new Robot(hardwareMap, telemetry);

        runtime.reset();
        waitForStart();


        Action bluered2 = drive.actionBuilder(new Pose2d(63.32,19.15, 0))

                .strafeToLinearHeading(new Vector2d(91.81,3.16), 0)
                .strafeToLinearHeading(new Vector2d(111.13,2.87), 0)
                .strafeToLinearHeading(new Vector2d(110.01,-6.24), 0)
                .strafeToLinearHeading(new Vector2d(69.92,-4.46), 0)
                .strafeToLinearHeading(new Vector2d(109.97,-5.29), 0)
                .strafeToLinearHeading(new Vector2d(110.64,-16.21), 0)
                .strafeToLinearHeading(new Vector2d(69.73,-15.41), 0)
                .strafeToLinearHeading(new Vector2d(111.23,-15.49), 0)
                .strafeToLinearHeading(new Vector2d(110.56,-21.25), 0)
                .strafeToLinearHeading(new Vector2d(68.25,-21.83), 0)
                .build();
////
//

        Actions.runBlocking(new SequentialAction(

                bluered2

        ));

        while (opModeIsActive()) {



            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //  Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
    }
}

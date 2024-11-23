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
                .strafeToLinearHeading(new Vector2d(90.72,45.70), 0)
                .strafeToLinearHeading(new Vector2d(89.92,3.49), 0)
                .strafeToLinearHeading(new Vector2d(115.20,1.84),0)
                .strafeToLinearHeading(new Vector2d(112.19,-8.11),0)
                .strafeToLinearHeading(new Vector2d(67.56,-7.30),0)
                .strafeToLinearHeading(new Vector2d(77.95,-7.70),0)
                .strafeToLinearHeading(new Vector2d(86.5,39.45),0)//
                .strafeToLinearHeading(new Vector2d(60.56,-7.30),0)
//                .strafeToLinearHeading(new Vector2d(75.95,-7.70),0)
                .strafeToLinearHeading(new Vector2d(84.88,33.39),0)//
                .strafeToLinearHeading(new Vector2d(60.56,-7.30),0)
//                .strafeToLinearHeading(new Vector2d(75.95,-7.70),0)
                .strafeToLinearHeading(new Vector2d(84.55,32.68),0)//
                .strafeToLinearHeading(new Vector2d(60.39,-10.62),0)
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

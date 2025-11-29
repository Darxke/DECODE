package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.MecanumDrive; // RR 1.0 quickstart drive

@Config
@Autonomous(name = "AutoFarRed", group = "Autonomous")
public class AutoFarRed extends LinearOpMode {
    private DcMotorEx rightOut;

    private DcMotorEx leftOut;
    private DcMotorEx intake;
    private Servo kick1;
    private Servo kick2;


    private static double open1 = 0.4;
    private static double close1 = 0;
    private static double open2 = 0.6;
    private static double close2 = 0.3;
    private static double push = 1;
    private static double store = 0.5;
    @Override
    public void runOpMode() {
        // Start pose — place robot here. Change if your field frame is different.
        rightOut = hardwareMap.get(DcMotorEx.class, "rightOut");
        leftOut = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        leftOut.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightOut.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightOut.setDirection(DcMotorSimple.Direction.REVERSE);
        Pose2d start = new Pose2d(59, 12, Math.toRadians(165));


        // Your RR 1.0 drive (ctor with start pose, like in your file)
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // === Path you requested ===

        Action shoot = drive.actionBuilder(new Pose2d(36,50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(59,12), Math.toRadians(165))
                .build();
        Action parking = drive.actionBuilder(new Pose2d(59,12, Math.toRadians(165)))
                .strafeToLinearHeading(new Vector2d(35,20), Math.toRadians(180))
                .build();
        Action out = drive.actionBuilder(new Pose2d(59,12, Math.toRadians(35)))
                .strafeToLinearHeading(new Vector2d(30,20), Math.toRadians(90))
                .build();
        Action cycle = drive.actionBuilder(new Pose2d(30, 20, Math.toRadians(90)))
                .strafeToLinearHeading(
                        new Vector2d(30, 50),
                        Math.toRadians(90),
                        new TranslationalVelConstraint(20.0),
                        new ProfileAccelConstraint(-10.0, 10.0)
                )
                .build();

        telemetry.addLine("Ready (RR 1.0). Set robot at start pose.");
        telemetry.update();
        waitForStart();
        leftOut.setVelocity(1450);
        rightOut.setVelocity(1450);
        intake.setPower(1);
        if (isStopRequested()) return;
        sleep(3500);
        shooting();
        sleep(1000);
        //    shooting();
        Actions.runBlocking(out);
        sleep(500);
        Actions.runBlocking(cycle);
        sleep(750);
        leftOut.setVelocity(1450);
        rightOut.setVelocity(1450);
        Actions.runBlocking(shoot);
        intake.setPower(1);
        shooting();
        sleep(1000);
        Actions.runBlocking(parking);
    }
    public void shooting(){
        kick1.setPosition(0);
        kick2.setPosition(1);
        sleep(1000);
        kick2.setPosition(0);
        sleep(1500);
        kick2.setPosition(1);
        sleep(1500);
        kick2.setPosition(0);
        sleep(1000);
        kick1.setPosition(1);
        sleep(1000);
        kick2.setPosition(1);
        sleep(500);
        kick1.setPosition(0);
        kick2.setPosition(0);
    }
}

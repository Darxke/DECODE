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
    private DcMotorEx outtake;
    private DcMotorEx intake;
    private Servo block1;
    private Servo block2;
    private Servo rotate;
    private Servo kicker;

    private static double open1 = 0.4;
    private static double close1 = 0;
    private static double open2 = 0.6;
    private static double close2 = 0.3;
    private static double push = 1;
    private static double store = 0.5;
    @Override
    public void runOpMode() {
        // Start pose — place robot here. Change if your field frame is different.
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        block1 = hardwareMap.get
                (Servo.class, "block1");
        block2 = hardwareMap.get(Servo.class, "block2");
        rotate = hardwareMap.get(Servo.class, "rotate");
        kicker = hardwareMap.get(Servo.class, "kicker");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Pose2d start = new Pose2d(59, 12, Math.toRadians(340));


        // Your RR 1.0 drive (ctor with start pose, like in your file)
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // === Path you requested ===

        Action shoot = drive.actionBuilder(new Pose2d(34.5,50, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(59,12), Math.toRadians(340))
                .build();
        Action parking = drive.actionBuilder(new Pose2d(59,12, Math.toRadians(35)))
                .strafeToLinearHeading(new Vector2d(35,20), Math.toRadians(0))
                .build();
        Action out = drive.actionBuilder(new Pose2d(59,12, Math.toRadians(35)))
                .strafeToLinearHeading(new Vector2d(33.5,20), Math.toRadians(90))
                .build();
        Action cycle = drive.actionBuilder(new Pose2d(33.5, 20, Math.toRadians(90)))
                .strafeToLinearHeading(
                        new Vector2d(34.5, 50),
                        Math.toRadians(90),
                        new TranslationalVelConstraint(20.0),
                        new ProfileAccelConstraint(-10.0, 10.0)
                )
                .build();

        telemetry.addLine("Ready (RR 1.0). Set robot at start pose.");
        telemetry.update();
        waitForStart();
        rotate.setPosition(0.03);
        outtake.setPower(0.65);
        if (isStopRequested()) return;
        sleep(3500);
        shooting();
        intake.setPower(1);
        rotate.setPosition(0.68);
        sleep(1000);
        //    shooting();
        Actions.runBlocking(out);
        sleep(500);
        Actions.runBlocking(cycle);
        sleep(750);
        kicker.setPosition(0.5);
        outtake.setPower(0.67);
        Actions.runBlocking(shoot);
        rotate.setPosition(1);
        sleep(1000);
        kicker.setPosition(1);
        sleep(500);
        kicker.setPosition(0.5);
        sleep(1000);
        rotate.setPosition(0.5);
        sleep(1000);
        kicker.setPosition(1);
        sleep(500);
        kicker.setPosition(0.5);
        sleep(1000);
        rotate.setPosition(0.03);
        sleep(1000);
        kicker.setPosition(1);
        rotate.setPosition(0.68);
        intake.setPower(1);
        sleep(1000);
        Actions.runBlocking(parking);
    }
    public void shooting(){
        kicker.setPosition(1);
        sleep(1000);
        kicker.setPosition(0.5);
        sleep(100);
        block2.setPosition(open2);
        sleep(1500);
        kicker.setPosition(1);
        sleep(1500);
        kicker.setPosition(0.5);
        sleep(1000);
        block1.setPosition(open1);
        sleep(1000);
        kicker.setPosition(1);
    }
}

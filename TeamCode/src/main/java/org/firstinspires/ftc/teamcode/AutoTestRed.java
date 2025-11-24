package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "RRCloseRed", group = "Autonomous")
public class AutoTestRed extends LinearOpMode {
    private DcMotorEx outtake;
    private DcMotorEx intake;
    private Servo block1;
    private Servo block2;
    private Servo rotate;
    private Servo kicker;

    // Tunables
    private static double open1 = 0.4;
    private static double close1 = 0.0;
    private static double open2 = 0.6;
    private static double close2 = 0.3;
    private static double push = 1.0;
    private static double store = 0.5;

    @Override
    public void runOpMode() {
        // Hardware
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        block1  = hardwareMap.get(Servo.class, "block1");
        block2  = hardwareMap.get(Servo.class, "block2");
        rotate  = hardwareMap.get(Servo.class, "rotate");
        kicker  = hardwareMap.get(Servo.class, "kicker");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // MIRRORED start pose across X-axis: (x, y, θ) -> (x, -y, -θ)
        Pose2d start = new Pose2d(-52, 52, Math.toRadians(-45));

        // RR 1.0 drive (ctor with start pose)
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // === Mirrored paths ===

        Action shoot = drive.actionBuilder(new Pose2d(-52, 52, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(-40))
                .build();

        Action cycle = drive.actionBuilder(new Pose2d(-20, 20, Math.toRadians(-40)))
                .splineTo(
                        new Vector2d(-14, 58),
                        Math.toRadians(90), // mirrored from 270 -> 90
                        new TranslationalVelConstraint(20.0),
                        new ProfileAccelConstraint(-10.0, 10.0)
                )
                .build();

        Action shoot2 = drive.actionBuilder(new Pose2d(-14, 58, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(-40))
                .build();

        Action cycle2 = drive.actionBuilder(new Pose2d(-20, 20, Math.toRadians(-40)))
                .splineTo(
                        new Vector2d(11, 34),       // y flipped from -34
                        Math.toRadians(95)          // mirrored from 265 -> 95
                )
                .waitSeconds(1)
                .strafeToConstantHeading(
                        new Vector2d(11, 58),       // y flipped from -58
                        new TranslationalVelConstraint(25.0),
                        new ProfileAccelConstraint(-10.0, 10.0)
                )
                .build();

        waitForStart();

        outtake.setPower(0.55);
        rotate.setPosition(0.03);
        if (isStopRequested()) return;

        telemetry.addLine("Ready (RR 1.0) — Red Close. Set robot at start pose.");
        telemetry.update();

        Actions.runBlocking(shoot);
        sleep(1750);
        shooting();

        intake.setPower(1);
        rotate.setPosition(0.68);
        sleep(1000);

        Actions.runBlocking(cycle);
        sleep(750);
        kicker.setPosition(0.5);
        outtake.setPower(0.55);

        Actions.runBlocking(shoot2);

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
        sleep(500);
        outtake.setPower(0);

        Actions.runBlocking(cycle2);
    }

    public void shooting() {
        kicker.setPosition(1);
        sleep(1000);
        kicker.setPosition(0.5);
        sleep(1000);
        block2.setPosition(open2);
        sleep(1000);
        kicker.setPosition(1);
        sleep(1000);
        kicker.setPosition(0.5);
        sleep(1000);
        block1.setPosition(open1);
        sleep(1000);
        kicker.setPosition(1);
    }
}

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
    private DcMotorEx rightOut;

    private DcMotorEx leftOut;
    private DcMotorEx intake;
    private Servo kick1;
    private Servo kick2;

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
        rightOut = hardwareMap.get(DcMotorEx.class, "rightOut");
        leftOut = hardwareMap.get(DcMotorEx.class, "leftOut");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");
        leftOut.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightOut.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightOut.setDirection(DcMotorSimple.Direction.REVERSE);

        // MIRRORED start pose across X-axis: (x, y, θ) -> (x, -y, -θ)
        Pose2d start = new Pose2d(-52, 52, Math.toRadians(-225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        Action shoot = drive.actionBuilder(new Pose2d(-52, 52, Math.toRadians(-225)))
                .strafeToConstantHeading(new Vector2d(-20, 20))
                .build();
        Action cycle = drive.actionBuilder(new Pose2d(-20, 20, Math.toRadians(-225)))
                .strafeToLinearHeading(new Vector2d(-11, 40), Math.toRadians(-265))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(-10, 62),
                        new TranslationalVelConstraint(25.0),
                        new ProfileAccelConstraint(-10.0, 10.0))
                .build();

        Action shoot2 = drive.actionBuilder(new Pose2d(-10, 48, Math.toRadians(-265)))
                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(-225))
                .build();

        Action cycle2 = drive.actionBuilder(new Pose2d(-20, 20, Math.toRadians(-225)))
                .strafeToLinearHeading(new Vector2d(12, 40), Math.toRadians(-265))

                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(11, 67),
                        new TranslationalVelConstraint(25.0),
                        new ProfileAccelConstraint(-10.0, 10.0))
                .build();


        waitForStart();

        leftOut.setVelocity(785);
        rightOut.setVelocity(785);
        if (isStopRequested()) return;
        intake.setPower(1);

        telemetry.addLine("Ready (RR 1.0) — Red Close. Set robot at start pose.");
        telemetry.update();
        Actions.runBlocking(shoot);
        sleep(500);
        shooting();
        sleep(500);
        //    shooting();
        Actions.runBlocking(cycle);
        sleep(750);
        leftOut.setVelocity(785);
        rightOut.setVelocity(785);
        Actions.runBlocking(shoot2);
        shooting();
        intake.setPower(1);
        sleep(500);
        leftOut.setPower(0);
        rightOut.setPower(0);
        Actions.runBlocking(cycle2);
    }
    public void shooting() {
        kick1.setPosition(0);
        kick2.setPosition(1);
        sleep(750);
        kick2.setPosition(0);
        sleep(1250);
        kick2.setPosition(1);
        sleep(750);
        kick2.setPosition(0);
        sleep(1250);
        kick1.setPosition(1);
        sleep(200);
        kick1.setPosition(1);
        kick2.setPosition(1);
        sleep(750);
        kick1.setPosition(0);
        kick2.setPosition(0);
    }
}

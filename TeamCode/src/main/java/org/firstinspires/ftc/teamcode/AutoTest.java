package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive; // RR 1.0 quickstart drive

@Config
@Autonomous(name = "RRAuto", group = "Autonomous")
public class AutoTest extends LinearOpMode {
    private DcMotorEx outtake;
    private DcMotorEx intake;
    private Servo block1;
    private Servo block2;
    private Servo rotate;

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
        block1 = hardwareMap.get(Servo.class, "block1");
        block2 = hardwareMap.get(Servo.class, "block2");
        rotate = hardwareMap.get(Servo.class, "rotate");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Pose2d start = new Pose2d(56, -16, Math.toRadians(210));


        // Your RR 1.0 drive (ctor with start pose, like in your file)
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // === Path you requested ===

        Action shoot = drive.actionBuilder(new Pose2d(56,-56, Math.toRadians(210)))
                .strafeToConstantHeading(new Vector2d(56,-16))
                .build();
        Action stock = drive.actionBuilder(new Pose2d(-56, -16, Math.toRadians(210)))
                .strafeToConstantHeading(new Vector2d(56, -56))
                .build();

        telemetry.addLine("Ready (RR 1.0). Set robot at start pose.");
        telemetry.update();
        rotate.setPosition(store);
        waitForStart();
        if (isStopRequested()) return;
        outtake.setPower(1);
        sleep(1500);
        rotate.setPosition(push);
        block1.setPosition(open1);
        sleep(500);
        //rotate
        //block 2
        //rotate
        Actions.runBlocking(stock);
        //rotate
        //block
        //block

        Actions.runBlocking(shoot);





    }
}

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
@Autonomous(name = "stanAuto", group = "Autonomous")
public class stanAuto extends LinearOpMode {
    private DcMotorEx outtake;
    private CRServo intakeM;
    @Override
    public void runOpMode() {
        // Start pose — place robot here. Change if your field frame is different.
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intakeM = hardwareMap.get(CRServo.class, "intakeM");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        Pose2d start = new Pose2d(56, -8, Math.toRadians(9.45));

        // Your RR 1.0 drive (ctor with start pose, like in your file)
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // === Path you requested ===

        Action path = drive.actionBuilder(new Pose2d(56, -8, Math.toRadians(9.45)))
                .strafeToLinearHeading(new Vector2d(-15, -8), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-56, -52), Math.toRadians(225))
                .build();

        telemetry.addLine("Ready (RR 1.0). Set robot at start pose.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Run the whole thing


        Actions.runBlocking(path);
    }
}

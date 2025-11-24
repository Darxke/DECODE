package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ATeleop")
public class ATeleop extends LinearOpMode {
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotorEx outtake;
    private DcMotorEx intake;
    private Servo block1;
    private Servo block2;
    private Servo rotate;
    private Servo kicker;

    private static double open1 = 0.4;
    private static double close1 = 0;
    private static double open2 = 0.8;
    private static double close2 = 0.3;
    private static double push = 1;
    private static double store = 0.4;
    boolean blockOpen = false;
    boolean block2Open = false;
    boolean stall = false;

    // ===== Outtake Variables =====
    private double outtakePower = 0.0;

    private static final double B_START_POWER = 0.55;
    private static final double START_START_POWER = 0.67;
    private static final double B_INCREMENT = 0.01;
    private static final double START_INCREMENT = 0.01;

    private boolean prevB = false;
    private boolean prevStart = false;
    private boolean prevX = false;

    @Override
    public void runOpMode() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        block1 = hardwareMap.get(Servo.class, "block1");
        block2 = hardwareMap.get(Servo.class, "block2");
        rotate = hardwareMap.get(Servo.class, "rotate");
        kicker = hardwareMap.get(Servo.class, "kicker");

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // ===== Drivetrain =====
                float LFspeed = gamepad1.left_stick_y -  gamepad1.left_stick_x - gamepad1.right_stick_x;
                float LBspeed = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
                float RFspeed = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                float RBspeed = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

                double maxSpeed = 1;
                if (gamepad1.left_bumper) {
                    maxSpeed = 0.4;
                } else if (gamepad1.right_bumper) {
                    maxSpeed = 0.3;
                }

                LFspeed = (float) Range.clip(LFspeed, -maxSpeed, maxSpeed);
                LBspeed = (float) Range.clip(LBspeed, -maxSpeed, maxSpeed);
                RFspeed = (float) Range.clip(RFspeed, -maxSpeed, maxSpeed);
                RBspeed = (float) Range.clip(RBspeed, -maxSpeed, maxSpeed);

                leftFront.setPower(LFspeed);
                leftBack.setPower(LBspeed);
                rightFront.setPower(RFspeed);
                rightBack.setPower(RBspeed);


                // ===== Kicker =====
                if (gamepad2.y) {
                    kicker.setPosition(1);
                } else {
                    if (!stall) {
                        kicker.setPosition(0.5);
                    }
                }

                if (gamepad2.back) {
                    intake.setPower(-1);
                }

                // ===== Block 1 Toggle =====
                if (gamepad2.right_bumper) {
                    blockOpen = !blockOpen;
                    if (blockOpen) block1.setPosition(open1);
                    else block1.setPosition(close1);

                    while (gamepad2.right_bumper && opModeIsActive()) idle();
                }

                // ===== Block 2 Toggle =====
                if (gamepad2.left_bumper) {
                    block2Open = !block2Open;
                    if (block2Open) block2.setPosition(open2);
                    else block2.setPosition(close2);

                    while (gamepad2.left_bumper && opModeIsActive()) idle();
                }


                // =====================================================================
                //                      OUTTAKE INCREMENT SYSTEM
                // =====================================================================

                // B → increase with base 0.55
                if (gamepad2.b && !prevB) {
                    if (outtakePower < B_START_POWER) {
                        outtakePower = B_START_POWER;
                    } else {
                        outtakePower += B_INCREMENT;
                    }
                }

                // Start → increase with base 0.65
                if (gamepad2.start && !prevStart) {
                    if (outtakePower < START_START_POWER) {
                        outtakePower = START_START_POWER;
                    } else {
                        outtakePower += START_INCREMENT;
                    }
                }

                // X → reset
                if (gamepad2.x && !prevX) {
                    outtakePower = 0.0;
                }

                outtakePower = Range.clip(outtakePower, -1.0, 1.0);
                outtake.setPower(outtakePower);

                // ===== TELEMETRY =====
                telemetry.addData("Outtake Power", outtakePower);
                telemetry.update();

                // update button states
                prevB = gamepad2.b;
                prevStart = gamepad2.start;
                prevX = gamepad2.x;


                // ===== Rotate + Intake =====
                if (gamepad2.dpad_down) {
                    intake.setPower(0.7);
                    rotate.setPosition(0.67);
                } else if (gamepad2.dpad_up) {
                    intake.setPower(0.7);
                    rotate.setPosition(0.03);
                } else if (gamepad2.dpad_right) {
                    intake.setPower(0.7);
                    rotate.setPosition(1);
                } else if (gamepad2.dpad_left) {
                    intake.setPower(0.7);
                    rotate.setPosition(0.5);
                }

                if (gamepad2.right_trigger > 0) {
                    rotate.setPosition(0.67);

                    stall = true;
                    kicker.setPosition(1);
                    intake.setPower(1);
                }

                if (gamepad2.left_trigger > 0) {
                    stall = false;
                    intake.setPower(0);
                }
            }
        }
    }
}

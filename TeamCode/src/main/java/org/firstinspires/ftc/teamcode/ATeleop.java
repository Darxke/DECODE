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

    private static double open1 = 0.4;
    private static double close1 = 0;
    private static double open2 = 0.8;
    private static double close2 = 0.3;
    private static double push = 1;
    private static double store = 0.4;
    boolean blockOpen = false;
    boolean block2Open = false;

    @Override
    public void runOpMode()  {
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront= hardwareMap.get(DcMotor.class, "rightFront");

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        block1 = hardwareMap.get(Servo.class, "block1");
        block2 = hardwareMap.get(Servo.class, "block2");
        rotate = hardwareMap.get(Servo.class, "rotate");
 // CRServo mapping

        // CRServo mapping

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive()){
                float LFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
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

                // ================== TOGGLES ==================
                // Right bumper: toggle both intakes + CRServo
//                if (gamepad2.right_bumper) {
//                    if ( intake.getPower() == 0) {
//                        intake.setPower(1.0);
//                    } else {
//                        intake.setPower(0);
//                    }
//                    while (opModeIsActive() && gamepad2.right_bumper) { idle(); }
//                }

                // B button: toggle outtake
                // inside loop
                if (gamepad2.y) {
                    rotate.setPosition(push);
                }

                else if (gamepad2.a){
                    rotate.setPosition(store);
                }
                if (gamepad2.right_bumper) {
                    blockOpen = !blockOpen;  // flip state each press

                    if (blockOpen) {
                        block1.setPosition(open1);   // open on first click
                    } else {
                        block1.setPosition(close1);  // close on second c+lick
                    }

                    // wait until the button is released so it doesn't toggle rapidly
                    while (gamepad2.right_bumper && opModeIsActive()) {
                        idle();
                    }
                }
                if (gamepad2.left_bumper){
                    block2Open = !block2Open;  // flip state each press

                    if (block2Open) {
                        block2.setPosition(open2);   // open on first click
                    } else {
                        block2.setPosition(close2);  // close on second click
                    }

                    while (gamepad2.left_bumper && opModeIsActive()) {
                        idle();
                    }
                }
                if (gamepad2.b) {
                    outtake.setPower(0.75);
                }
                if (gamepad2.x){
                    outtake.setPower(0.0);
                }


// DO NOT block with while(...gamepad2.b). Let the loop iterate.

                // ==============================================
            }
        }
    }
}

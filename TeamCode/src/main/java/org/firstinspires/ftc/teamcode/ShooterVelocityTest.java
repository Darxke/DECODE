package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ShooterVelocityTestTwoMotors")
public class ShooterVelocityTest extends LinearOpMode {

    private DcMotorEx leftOuttake;
    private DcMotorEx rightOuttake;

    private double targetVelocity = 1500;          // ticks per second
    private static final double STEP = 100;        // change per button press
    private static final double MIN_VEL = 0;
    private static final double MAX_VEL = 5000;    // tune for your motor
    private static final double TICKS_PER_REV = 28.0; // change if needed

    @Override
    public void runOpMode() throws InterruptedException {

        leftOuttake = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightOuttake = hardwareMap.get(DcMotorEx.class, "rightOuttake");

        // Must use encoders for velocity
        leftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Flip one if needed so wheels spin the same direction
        leftOuttake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean lastX = false;
        boolean lastB = false;

        telemetry.addLine("ShooterVelocityTestTwoMotors Ready");
        telemetry.addLine("B = increase velocity");
        telemetry.addLine("X = decrease velocity");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean x = gamepad1.x;
            boolean b = gamepad1.b;

            // Edge-detect button presses
            if (b && !lastB) targetVelocity += STEP;
            if (x && !lastX) targetVelocity -= STEP;

            targetVelocity = Range.clip(targetVelocity, MIN_VEL, MAX_VEL);

            // Apply same velocity to both
            leftOuttake.setVelocity(targetVelocity);
            rightOuttake.setVelocity(targetVelocity);

            // Read current velocities
            double leftVel = leftOuttake.getVelocity();   // ticks / second
            double rightVel = rightOuttake.getVelocity(); // ticks / second

            double leftRPM = (leftVel / TICKS_PER_REV) * 60.0;
            double rightRPM = (rightVel / TICKS_PER_REV) * 60.0;

            telemetry.addData("Target Velocity (tps)", targetVelocity);
            telemetry.addData("Left  Vel (tps)", leftVel);
            telemetry.addData("Left  RPM", leftRPM);
            telemetry.addData("Right Vel (tps)", rightVel);
            telemetry.addData("Right RPM", rightRPM);

            if (leftVel == 0 || rightVel == 0) {
                telemetry.addLine("⚠ One or both velocities = 0 → check encoders/config.");
            }

            telemetry.update();

            lastB = b;
            lastX = x;
        }
    }
}

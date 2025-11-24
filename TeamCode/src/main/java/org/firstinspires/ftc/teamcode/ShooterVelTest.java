package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ShooterVelTest")
public class ShooterVelTest extends LinearOpMode {

    private DcMotorEx outtake;
    private Servo kicker;
    private double targetVelocity = 1000;
    private static final double STEP = 100;
    private static final double MIN_VEL = 0;
    private static final double MAX_VEL = 5000;   // safer max
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        // MUST BE USING ENCODER TO GET VELOCITY
        outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);

        kicker = hardwareMap.get(Servo.class, "kicker");

        boolean lastX = false;
        boolean lastB = false;

        waitForStart();

        while (opModeIsActive()) {

            boolean x = gamepad1.x;
            boolean b = gamepad1.b;

            if (b && !lastB) targetVelocity += STEP;
            if (x && !lastX) targetVelocity -= STEP;

            targetVelocity = Range.clip(targetVelocity, MIN_VEL, MAX_VEL);

            outtake.setVelocity(targetVelocity);

            double currentVel = outtake.getVelocity();  // <-- THIS IS THE REAL VALUE
            double rpm = (currentVel / TICKS_PER_REV) * 60.0;

            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity (tps)", currentVel);
            telemetry.addData("Current RPM", rpm);

            if (currentVel == 0)
                telemetry.addLine("⚠ Velocity = 0 → Encoder not reading!");
            if (currentVel == targetVelocity && gamepad1.y) {
                telemetry.addLine("✔ Encoder is reading target");
                kicker.setPosition(1);
            }
            telemetry.update();

            lastB = b;
            lastX = x;
        }
    }
}

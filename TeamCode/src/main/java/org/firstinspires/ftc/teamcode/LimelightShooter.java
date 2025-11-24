package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "LimelightShooterVelocity", group = "Test")
public class LimelightShooter extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotorEx outtake;

    // CAMERA → for distance (floor -> camera lens)
    private static final double CAMERA_HEIGHT_IN = 15.0;
    // floor -> AprilTag CENTER (you measured this)
    private static final double APRILTAG_HEIGHT_IN = 29.75;
    // camera tilt (0 if actually level)
    private static final double CAMERA_MOUNT_ANGLE_DEG = 0.0;

    // SHOOTER → for projectile to GOAL
    private static final double SHOOTER_HEIGHT_IN = 8.0;   // floor -> shooter exit
    private static final double GOAL_HEIGHT_IN    = 45.0;  // floor -> goal center
    private static final double SHOOTER_ANGLE_DEG = 60.0;  // shooter angle above horizontal

    // shooter and camera horizontally aligned (change if not true)
    private static final double SHOOTER_FORWARD_OFFSET_IN = 0.0;

    // we want to shoot 12" past the tag distance
    private static final double EXTRA_SHOOT_DIST_IN = 12.0;

    // wheel and motor
    private static final double WHEEL_RADIUS_IN = 1.5;    // shooter wheel radius in inches
    private static final double MAX_RPM = 6000.0;         // motor free speed approx
    private static final double TICKS_PER_REV = 28.0;     // 5202 6000 rpm Yellow Jacket

    // gravity in inches/s^2
    private static final double G_INCHES = 386.08858;

    // tuning to fix "power is too low"
    private static final double RPM_SCALE = 1.8;   // >1 = more oomph
    private static final double MIN_RPM   = 2500;  // floor RPM when actually shooting

    @Override
    public void runOpMode() throws InterruptedException {
        // hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD); // flip if backwards
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.setMsTransmissionInterval(11);

        // use your AprilTag pipeline index
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("LimelightShooterVelocity: ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            boolean hasTarget = false;
            double tx = 0.0;
            double ty = 0.0;

            double camDistInches = 0.0;
            double shooterDistInches = 0.0;
            double aimDistInches = 0.0;

            double requiredVelInPerSec = Double.NaN;
            double requiredRpm = Double.NaN;
            double targetTicksPerSec = 0.0;
            double equivPower = 0.0;

            if (result != null && result.isValid()) {
                hasTarget = true;

                tx = result.getTx();
                ty = result.getTy();

                // 1) distance from CAMERA to TAG using ty and tag height
                camDistInches = calculateCameraDistanceInches(ty);

                // 2) shooter distance to TAG (horizontal)
                shooterDistInches = camDistInches - SHOOTER_FORWARD_OFFSET_IN;
                if (shooterDistInches < 0) shooterDistInches = 0;

                // 3) AIM distance = tag distance + 12 inches
                aimDistInches = shooterDistInches + EXTRA_SHOOT_DIST_IN;

                // 4) required ball exit speed to hit GOAL from AIM distance
                requiredVelInPerSec = getRequiredExitVelocity(aimDistInches);

                if (!Double.isNaN(requiredVelInPerSec)) {
                    // 5) convert to wheel RPM from physics
                    requiredRpm = velocityToWheelRpm(requiredVelInPerSec, WHEEL_RADIUS_IN);

                    // scale up to account for drag, slip, etc.
                    requiredRpm *= RPM_SCALE;

                    // enforce minimum useful RPM when actually shooting
                    if (requiredRpm > 0 && requiredRpm < MIN_RPM) {
                        requiredRpm = MIN_RPM;
                    }

                    // clamp to motor's max
                    if (requiredRpm < 0) requiredRpm = 0;
                    if (requiredRpm > MAX_RPM) requiredRpm = MAX_RPM;

                    // 6) convert RPM to ticks/s for DcMotorEx.setVelocity()
                    targetTicksPerSec = rpmToTicksPerSecond(requiredRpm, TICKS_PER_REV);

                    outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outtake.setVelocity(targetTicksPerSec);

                    // 7) equivalent power (for debug)
                    equivPower = requiredRpm / MAX_RPM;
                    if (Double.isNaN(equivPower)) equivPower = 0;
                    if (equivPower < 0) equivPower = 0;
                    if (equivPower > 1) equivPower = 1;
                } else {
                    // geometry says impossible shot (too close or angle wrong)
                    outtake.setVelocity(0);
                    targetTicksPerSec = 0;
                    equivPower = 0;
                }
            } else {
                // no tag detected
                outtake.setVelocity(0);
                equivPower = 0;
            }

            telemetry.addData("Has Target", hasTarget);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Cam Dist to Tag (in)", camDistInches);
            telemetry.addData("Shoot Dist to Tag (in)", shooterDistInches);
            telemetry.addData("Aim Dist (tag+12 in)", aimDistInches);
            telemetry.addData("Req v (in/s)", requiredVelInPerSec);
            telemetry.addData("Req RPM (scaled)", requiredRpm);
            telemetry.addData("Target Vel (ticks/s)", targetTicksPerSec);
            telemetry.addData("Equiv Power (0–1)", equivPower);
            telemetry.update();

            sleep(20);
        }

        outtake.setVelocity(0);
    }

    /**
     * Distance from camera to AprilTag using Limelight ty:
     * d = (h_tag - h_cam) / tan(a_cam + ty)
     */
    private double calculateCameraDistanceInches(double tyDeg) {
        double angleToTargetDeg = CAMERA_MOUNT_ANGLE_DEG + tyDeg;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);

        return (APRILTAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleToTargetRad);
    }

    /**
     * Required exit velocity (in/s) to hit GOAL from horizontal distance d.
     * v^2 = g d^2 / (2 cos^2(theta) (d tan(theta) - Δh))
     */
    private double getRequiredExitVelocity(double dInches) {
        double thetaRad = Math.toRadians(SHOOTER_ANGLE_DEG);
        double deltaH = GOAL_HEIGHT_IN - SHOOTER_HEIGHT_IN;

        double numerator = G_INCHES * dInches * dInches;
        double denomInside = dInches * Math.tan(thetaRad) - deltaH;

        if (denomInside <= 0) {
            // shooter angle too low or distance too short for this geometry
            return Double.NaN;
        }

        double denominator = 2.0
                * Math.cos(thetaRad) * Math.cos(thetaRad)
                * denomInside;

        return Math.sqrt(numerator / denominator); // inches per second
    }

    /**
     * Convert ball speed (in/s) to wheel RPM, assuming rim speed ~ ball speed.
     */
    private double velocityToWheelRpm(double vInchesPerSec, double wheelRadiusInches) {
        if (wheelRadiusInches <= 0) return Double.NaN;
        double omegaRadPerSec = vInchesPerSec / wheelRadiusInches;
        return omegaRadPerSec * 60.0 / (2.0 * Math.PI);
    }

    /**
     * Convert RPM to encoder ticks per second for DcMotorEx.setVelocity().
     */
    private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }
}

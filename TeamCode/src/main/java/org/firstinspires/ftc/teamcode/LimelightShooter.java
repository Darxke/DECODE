package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LimelightShooter", group = "Test")
public class LimelightShooter extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotorEx leftOuttake;
    private DcMotorEx rightOuttake;

    // New: servos
    private Servo kick1;
    private Servo kick2;

    // CAMERA → for distance (floor -> camera lens)
    private static final double CAMERA_HEIGHT_IN = 11.0;      // camera height
    // floor -> AprilTag CENTER
    private static final double APRILTAG_HEIGHT_IN = 29.75;
    // camera tilt upward (degrees)
    private static final double CAMERA_MOUNT_ANGLE_DEG = 5.0; // tilted up 5 degrees

    // SHOOTER + HOOD → projectile to GOAL
    private static final double SHOOTER_HEIGHT_IN = 13.0;     // floor -> ball exit point
    private static final double GOAL_HEIGHT_IN    = 45.0;     // floor -> goal center
    private static final double SHOOTER_ANGLE_DEG = 60.0;     // hood / exit angle up

    // horizontal offset: shooter is 10" behind camera
    private static final double SHOOTER_FORWARD_OFFSET_IN = 10.0;

    // how far past the tag we aim
    private static final double EXTRA_SHOOT_DIST_IN = 8.0;

    // wheel and motor
    private static final double WHEEL_RADIUS_IN = 1.5;
    private static final double MAX_RPM = 6000.0;
    private static final double TICKS_PER_REV = 28.0; // 5202 6000 rpm

    // gravity in inches/s^2
    private static final double G_INCHES = 386.08858;

    // optional floor; still zero for now
    private static final double MIN_RPM = 0.0;

    // smoothing
    private static final double RPM_SMOOTH_ALPHA = 0.3;
    private static final double MAX_RPM_STEP     = 400.0;

    // stability / ready
    private double lastAimDist = Double.NaN;
    private int stableFrames = 0;

    // RPM smoothing
    private double lastRequiredRpm = Double.NaN;

    // Servo positions (you asked: 0 and 1)
    private static final double KICK_REST_POS = 0.0;
    private static final double KICK_FIRE_POS = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // hardware
        limelight    = hardwareMap.get(Limelight3A.class, "limelight");
        leftOuttake  = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightOuttake = hardwareMap.get(DcMotorEx.class, "rightOuttake");

        // servos
        kick1 = hardwareMap.get(Servo.class, "kick1");
        kick2 = hardwareMap.get(Servo.class, "kick2");

        // initialize servos to rest
        kick1.setPosition(KICK_REST_POS);
        kick2.setPosition(KICK_REST_POS);

        leftOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOuttake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.setMsTransmissionInterval(11);

        // AprilTag pipeline index
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("LimelightShooter: ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ====== GAMEPAD CONTROL FOR SERVOS ======
            // kick1: A = fire, B = rest
            if (gamepad1.a) {
                kick1.setPosition(KICK_FIRE_POS);
            } else if (gamepad1.b) {
                kick1.setPosition(KICK_REST_POS);
            }

            // kick2: X = fire, Y = rest
            if (gamepad1.x) {
                kick2.setPosition(KICK_FIRE_POS);
            } else if (gamepad1.y) {
                kick2.setPosition(KICK_REST_POS);
            }

            // ====== LIMELIGHT & SHOOTER ======
            LLResult result = limelight.getLatestResult();

            boolean hasTarget = false;
            double tx = 0.0;
            double ty = 0.0;

            double camDistInches = 0.0;
            double shooterDistInches = 0.0;
            double aimDistInches = Double.NaN;

            double requiredVelInPerSec = Double.NaN;
            double requiredRpmRaw = Double.NaN;
            double requiredRpm = Double.NaN;
            double targetTicksPerSec = 0.0;
            double equivPower = 0.0;
            double rpmScale = 1.0;
            boolean shooterReady = false;

            if (result != null && result.isValid()) {
                hasTarget = true;

                tx = result.getTx();
                ty = result.getTy();

                // 1) distance from CAMERA to TAG
                camDistInches = calculateCameraDistanceInches(ty);

                // 2) shooter distance to TAG (horizontal)
                shooterDistInches = camDistInches + SHOOTER_FORWARD_OFFSET_IN;
                if (shooterDistInches < 0) shooterDistInches = 0;

                // 3) AIM distance = shooter->tag + extra
                aimDistInches = shooterDistInches + EXTRA_SHOOT_DIST_IN;

                // 4) required exit velocity
                requiredVelInPerSec = getRequiredExitVelocity(aimDistInches);

                if (!Double.isNaN(requiredVelInPerSec)) {
                    // 5) base RPM from physics
                    requiredRpmRaw = velocityToWheelRpm(requiredVelInPerSec, WHEEL_RADIUS_IN);

                    // 6) conditional scale:
                    // front (<= 80 in): 1.5
                    // back  (> 80 in): 1.3
                    if (!Double.isNaN(aimDistInches) && aimDistInches <= 80.0) {
                        rpmScale = 1.5;
                    } else {
                        rpmScale = 1.3;
                    }
                    double scaledRpm = requiredRpmRaw * rpmScale;

                    // clamp
                    if (scaledRpm < 0) scaledRpm = 0;
                    if (scaledRpm > MAX_RPM) scaledRpm = MAX_RPM;

                    // 7) smoothing
                    if (Double.isNaN(lastRequiredRpm)) {
                        requiredRpm = scaledRpm;
                    } else {
                        double blended = lastRequiredRpm
                                + RPM_SMOOTH_ALPHA * (scaledRpm - lastRequiredRpm);

                        double delta = blended - lastRequiredRpm;
                        if (delta > MAX_RPM_STEP) delta = MAX_RPM_STEP;
                        if (delta < -MAX_RPM_STEP) delta = -MAX_RPM_STEP;

                        requiredRpm = lastRequiredRpm + delta;
                    }
                    lastRequiredRpm = requiredRpm;

                    // min clamp if you ever set MIN_RPM > 0
                    if (requiredRpm > 0 && requiredRpm < MIN_RPM) {
                        requiredRpm = MIN_RPM;
                    }

                    if (requiredRpm < 0) requiredRpm = 0;
                    if (requiredRpm > MAX_RPM) requiredRpm = MAX_RPM;

                    // 8) apply to motors
                    targetTicksPerSec = rpmToTicksPerSecond(requiredRpm, TICKS_PER_REV);

                    leftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    leftOuttake.setVelocity(targetTicksPerSec);
                    rightOuttake.setVelocity(targetTicksPerSec);

                    // equivalent power
                    equivPower = requiredRpm / MAX_RPM;
                    if (Double.isNaN(equivPower)) equivPower = 0;
                    if (equivPower < 0) equivPower = 0;
                    if (equivPower > 1) equivPower = 1;

                    // 9) stability / "ready"
                    if (!Double.isNaN(aimDistInches)) {
                        if (!Double.isNaN(lastAimDist)
                                && Math.abs(aimDistInches - lastAimDist) < 2.0) {
                            stableFrames++;
                        } else {
                            stableFrames = 0;
                        }
                        lastAimDist = aimDistInches;

                        if (stableFrames >= 5 && hasTarget && requiredRpm > 0) {
                            shooterReady = true;
                        } else {
                            shooterReady = false;
                        }
                    } else {
                        stableFrames = 0;
                        shooterReady = false;
                    }
                } else {
                    stopShooters();
                    targetTicksPerSec = 0;
                    equivPower = 0;
                    stableFrames = 0;
                    shooterReady = false;
                    lastRequiredRpm = Double.NaN;
                }
            } else {
                stopShooters();
                equivPower = 0;
                stableFrames = 0;
                shooterReady = false;
                lastRequiredRpm = Double.NaN;
            }

            telemetry.addData("Has Target", hasTarget);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Cam Dist to Tag (in)", camDistInches);
            telemetry.addData("Shoot Dist to Tag (in)", shooterDistInches);
            telemetry.addData("Aim Dist (tag+offset, in)", aimDistInches);
            telemetry.addData("Req v (in/s)", requiredVelInPerSec);
            telemetry.addData("Raw RPM (physics)", requiredRpmRaw);
            telemetry.addData("RPM Scale", rpmScale);
            telemetry.addData("Req RPM (smoothed)", requiredRpm);
            telemetry.addData("Target Vel (ticks/s)", targetTicksPerSec);
            telemetry.addData("Equiv Power (0–1)", equivPower);
            telemetry.addData("Stable Frames", stableFrames);
            telemetry.addData("Shooter Ready", shooterReady);
            telemetry.addData("Kick1 Pos", kick1.getPosition());
            telemetry.addData("Kick2 Pos", kick2.getPosition());
            telemetry.update();

            sleep(20);
        }

        stopShooters();
    }

    private void stopShooters() {
        leftOuttake.setVelocity(0);
        rightOuttake.setVelocity(0);
    }

    // Distance from camera to AprilTag using Limelight ty
    private double calculateCameraDistanceInches(double tyDeg) {
        double angleToTargetDeg = CAMERA_MOUNT_ANGLE_DEG + tyDeg;
        double angleToTargetRad = Math.toRadians(angleToTargetDeg);
        return (APRILTAG_HEIGHT_IN - CAMERA_HEIGHT_IN) / Math.tan(angleToTargetRad);
    }

    // Required exit velocity (in/s) to hit GOAL from horizontal distance d
    private double getRequiredExitVelocity(double dInches) {
        double thetaRad = Math.toRadians(SHOOTER_ANGLE_DEG);
        double deltaH = GOAL_HEIGHT_IN - SHOOTER_HEIGHT_IN;

        double numerator = G_INCHES * dInches * dInches;
        double denomInside = dInches * Math.tan(thetaRad) - deltaH;

        if (denomInside <= 0) {
            return Double.NaN;
        }

        double denominator = 2.0
                * Math.cos(thetaRad) * Math.cos(thetaRad)
                * denomInside;

        return Math.sqrt(numerator / denominator);
    }

    // Convert ball speed (in/s) to wheel RPM
    private double velocityToWheelRpm(double vInchesPerSec, double wheelRadiusInches) {
        if (wheelRadiusInches <= 0) return Double.NaN;
        double omegaRadPerSec = vInchesPerSec / wheelRadiusInches;
        return omegaRadPerSec * 60.0 / (2.0 * Math.PI);
    }

    // Convert RPM to encoder ticks per second
    private double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }
}

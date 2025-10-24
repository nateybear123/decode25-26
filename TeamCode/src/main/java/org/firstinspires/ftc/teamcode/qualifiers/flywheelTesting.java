package org.firstinspires.ftc.teamcode.qualifiers;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

@TeleOp(name = "Flywheel Testing", group = "qualifiers")
public class flywheelTesting extends LinearOpMode {

    //todo this is the target id that it is looking for, this is set for the blue goal rn
    private static final int TARGET_TAG_ID = 20;

    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int CAM_W = 640, CAM_H = 480;
    private static final double TAG_SIZE_M = 0.2065;        //in meters
    private static final AprilTagProcessor.TagFamily TAG_FAMILY = AprilTagProcessor.TagFamily.TAG_36h11;
    private static final double C270_HFOV_DEG = 60.0;

    //todo edit these values
    private static final double H_GOAL_M    = 1.016;   // goal center height (m)
    private static final double H_LAUNCH_M  = 0.30;   // shooter exit height (m)
    private static final double THETA_DEG   = 38.0;   // shooter angle above horizontal (deg)
    private static final double KEFF        = 0.80;   // exit_speed ≈ KEFF * wheel_surface_speed

    private static final String FLYWHEEL_MOTOR = "flywheel";
    private static final double WHEEL_DIAMETER_M = 0.090;
    private static final double GEAR_RATIO_WHEEL_PER_MOTOR = 1.0;
    private static final double TICKS_PER_REV = 28;        

    private static final double MIN_RANGE_M = 0.25, MAX_RANGE_M = 5.0;
    private static final double MIN_RPM = 500.0, MAX_RPM = 7000.0;
    private static final double SPEED_TOL_RPM = 120.0;            // for telemetry
    private static final long   ONE_SHOT_HOLD_MS = 800;           // time to hold at speed
    private static final long   ONE_SHOT_SPINUP_TIMEOUT_MS = 2000;// give up if can't reach speed

    //smoothing for range
    private static final int RANGE_SMA_WINDOW = 5;
    private final Deque<Double> rangeWindow = new ArrayDeque<>();

    //create the objects
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private DcMotorEx flywheel;

    //these are the states
    private boolean spinningHold = false;  // LB to keep spinning
    private boolean prevRB = false;
    private boolean oneShotActive = false;
    private long    oneShotStartMs = 0L;   // when we hit at-speed
    private long    oneShotSpinupStartMs = 0L;

    private double targetRpm = 0.0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        //init stuff
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        initAprilTag();

        telemetry.addLine("Controls: LB=hold spin, RB=one-shot, B=stop");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            final boolean lb = gamepad1.left_bumper;
            final boolean rb = gamepad1.right_bumper;
            final boolean bStop = gamepad1.b;

            // B always cancels everything
            if (bStop) {
                spinningHold = false;
                oneShotActive = false;
                stopFlywheel();
            }

            // LB logic: keep spinning and continuously update target
            if (lb) {
                double distanceM = getSmoothedRangeMeters();
                double rpm = computeTargetRPM(distanceM);
                targetRpm = clamp(rpm, MIN_RPM, MAX_RPM);
                spinTo(targetRpm);
                spinningHold = true;

                // If LB pressed during a one-shot, cancel one-shot and change to spinning
                oneShotActive = false;
            }


            // RB: start a one-shot if not already one-shot and not LB
            if (rb && !prevRB && !lb) {
                double distanceM = getSmoothedRangeMeters();
                double rpm = computeTargetRPM(distanceM);
                targetRpm = clamp(rpm, MIN_RPM, MAX_RPM);
                spinTo(targetRpm);
                oneShotActive = true;
                oneShotStartMs = 0L;              // not at speed yet
                oneShotSpinupStartMs = System.currentTimeMillis();
            }
            prevRB = rb;

            if (oneShotActive) {
                // If LB gets pressed mid one-shot, LB takes over above

                // Abort one-shot if spinup taking too long
                if (System.currentTimeMillis() - oneShotSpinupStartMs > ONE_SHOT_SPINUP_TIMEOUT_MS) {
                    oneShotActive = false;
                    stopFlywheel();
                } else {
                    // Keep commanding target RPM
                    spinTo(targetRpm);

                    if (oneShotStartMs == 0L) {
                        // Waiting to reach speed
                        if (atSpeed(targetRpm)) {
                            oneShotStartMs = System.currentTimeMillis();
                        }
                    } else {
                        if (System.currentTimeMillis() - oneShotStartMs >= ONE_SHOT_HOLD_MS) {
                            oneShotActive = false;
                            stopFlywheel();
                        }
                    }
                }
            }

            // If neither LB nor one-shot is active and hold mode isn't sticky, stop
            if (!lb && !oneShotActive && !spinningHold) {
                stopFlywheel();
            }

            // Telemetry
            telemetry.addData("Hold Mode (LB)", spinningHold);
            telemetry.addData("One-Shot (RB)", oneShotActive);
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Measured RPM", "%.0f", ticksPerSecToRpm(flywheel.getVelocity()));
            telemetry.addData("Range (m)", rangeWindow.isEmpty() ? "N/A"
                    : String.format("%.2f", last(rangeWindow)));

            boolean sawTarget = false;
            for (AprilTagDetection d : aprilTag.getDetections()) {
                if (d.id == TARGET_TAG_ID) { sawTarget = true; break; }
            }
            telemetry.addData("Tag 20 visible", sawTarget);
            telemetry.update();

            sleep(20);
        }

        if (visionPortal != null) visionPortal.close();
    }

    // ===== Vision setup =====
    private void initAprilTag() {
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
                .setTagFamily(TAG_FAMILY)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true);
        aprilTag = tagBuilder.build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(CAM_W, CAM_H))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();
    }

    private double getSmoothedRangeMeters() {
        List<AprilTagDetection> dets = aprilTag.getDetections();
        if (dets.isEmpty()) return fallbackRange();

        // find the specific tag we care about
        AprilTagDetection target = null;
        for (AprilTagDetection d : dets) {
            if (d.id == TARGET_TAG_ID) {
                target = d;
                break;
            }
        }
        if (target == null) return fallbackRange();
        double r = estimateRangeMetersFromPixels(target);

        if (r <= 0 || r < MIN_RANGE_M || r > MAX_RANGE_M) return fallbackRange();

        rangeWindow.addLast(r);
        if (rangeWindow.size() > RANGE_SMA_WINDOW) rangeWindow.removeFirst();

        double sum = 0;
        for (double x : rangeWindow) sum += x;
        return sum / rangeWindow.size();
    }

    private double fallbackRange() {
        return rangeWindow.isEmpty() ? 1.2 : last(rangeWindow);
    }

    private static double last(java.util.Deque<Double> dq) {
        Double v = dq.peekLast();
        return (v != null) ? v : -1;
    }

    private double computeTargetRPM(double dMeters) {
        final double G = 9.80665;
        double theta = Math.toRadians(THETA_DEG);
        double h = H_GOAL_M - H_LAUNCH_M;

        double denom = 2.0 * Math.pow(Math.cos(theta), 2) * (dMeters * Math.tan(theta) - h);
        if (denom < 0.05) denom = 0.05; // guard near geometric edge cases

        double vExit = Math.sqrt((G * dMeters * dMeters) / denom); // m/s

        double wheelRPM = (60.0 * vExit) / (KEFF * Math.PI * WHEEL_DIAMETER_M);
        double motorRPM = wheelRPM / GEAR_RATIO_WHEEL_PER_MOTOR;
        return clamp(motorRPM, MIN_RPM, MAX_RPM);
    }

    private void spinTo(double rpm) {
        flywheel.setVelocity(rpmToTicksPerSec(rpm)); // RUN_USING_ENCODER velocity loop
    }

    private void stopFlywheel() {
        flywheel.setPower(0.0);
        targetRpm = 0.0;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * (TICKS_PER_REV / 60.0);
    }

    private static double ticksPerSecToRpm(double tps) {
        return tps * (60.0 / TICKS_PER_REV);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private boolean atSpeed(double rpmTarget) {
        double rpmNow = ticksPerSecToRpm(flywheel.getVelocity());
        return Math.abs(rpmNow - rpmTarget) <= SPEED_TOL_RPM;
    }
    // Estimates focal length in pixels from horizontal FOV and image width.
    private double fxPixels() {
        // fx = (w/2) / tan(hfov/2)
        return (CAM_W / 2.0) / Math.tan(Math.toRadians(C270_HFOV_DEG / 2.0));
    }

    // Compute the detected tag's pixel width from its 4 corners.
    private static double pixelWidthOfTag(AprilTagDetection d) {
        // Use average of top and bottom edges for robustness
        double dxTop  = d.corners[1].x - d.corners[0].x;
        double dyTop  = d.corners[1].y - d.corners[0].y;
        double dxBot  = d.corners[2].x - d.corners[3].x;
        double dyBot  = d.corners[2].y - d.corners[3].y;
        double wTop   = Math.hypot(dxTop, dyTop);
        double wBot   = Math.hypot(dxBot, dyBot);
        return 0.5 * (wTop + wBot);
    }

    private double estimateRangeMetersFromPixels(AprilTagDetection d) {
        double pxW = pixelWidthOfTag(d);
        if (pxW <= 1e-3) return -1;
        return (TAG_SIZE_M * fxPixels()) / pxW;
    }
}
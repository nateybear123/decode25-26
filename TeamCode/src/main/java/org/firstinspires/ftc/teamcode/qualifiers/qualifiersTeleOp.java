package org.firstinspires.ftc.teamcode.qualifiers;



import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.graphics.Color;

@TeleOp(name = "TeleOp", group = "Qualifiers")
public class qualifiersTeleOp extends LinearOpMode{

    private static final double DEADBAND = 0.05;            // ignore tiny stick noise
    private static final double INPUT_CURVE_EXP = 3.0;      // 3 = cubic response
    private static final double MAX_SLEW_PER_SEC = 4.0;     // max power change per second
    private static final double NORMAL_SPEED = 0.75;
    private static final double SLOW_SPEED   = 0.45;
    private static final double FAST_SPEED   = 1.00;

    private static final double ROTATION_WEIGHT = 1.0;


    private NormalizedColorSensor colorSensor;
    private SwitchableLight light;
    private final float[] hsv = new float[3];

    private static final float GREEN_MIN = 85f,  GREEN_MAX = 160f;
    private static final float PURPLE_MIN = 260f, PURPLE_MAX = 320f;

    private static final float MIN_SAT = 0.25f;
    private static final float MIN_VAL = 0.15f;

    private enum ColorClass { GREEN, PURPLE, UNKNOWN }
    qualifiersHardwareMap hardware = new qualifiersHardwareMap();

    private double prevFL = 0, prevFR = 0, prevBL = 0, prevBR = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (colorSensor instanceof SwitchableLight) {
            light = (SwitchableLight) colorSensor;
            light.enableLight(true);
        }

        telemetry.addLine("Simple Green/Purple classifier ready");
        telemetry.update();

        // ====== Bulk caching to reduce bus traffic and jitter ======
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // ====== Motors ======
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        // Standard mecanum direction: reverse the left side so +power drives forward
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // Use encoders with built-in velocity control for consistent feel
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ====== IMU (generic) ======
        // Adjust these if your Control Hub is mounted differently.
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);
        imu.resetYaw();

        telemetry.addLine("Initialized. Press ▶ to start");
        telemetry.addLine("Y = reset heading | LB = slow | RB = fast");
        telemetry.update();

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        while (opModeIsActive()){
            detectColor();
            mechanumDrive();
        }


    }

    private void detectColor(){
        NormalizedRGBA c = colorSensor.getNormalizedColors();
        int r = to8bit(c.red), g = to8bit(c.green), b = to8bit(c.blue);
        Color.RGBToHSV(r, g, b, hsv);
        float hue = hsv[0], sat = hsv[1], val = hsv[2];

        ColorClass detected = classify(hue, sat, val);

        switch (detected) {
            case GREEN:
                // TODO: replace with your real action for GREEN
                telemetry.addLine("Action: GREEN detected");
                break;
            case PURPLE:
                // TODO: replace with your real action for PURPLE
                telemetry.addLine("Action: PURPLE detected");
                break;
            default:
                // No action for UNKNOWN
                telemetry.addLine("Action: none (Unknown)");
                break;
        }
    }
    private int to8bit(float norm) {
        return Math.min(255, Math.max(0, (int) (norm * 255f + 0.5f)));
    }

    private ColorClass classify(float hue, float sat, float val) {
        if (sat < MIN_SAT || val < MIN_VAL) return ColorClass.UNKNOWN;

        if (hue >= GREEN_MIN && hue <= GREEN_MAX)   return ColorClass.GREEN;
        if (hue >= PURPLE_MIN && hue <= PURPLE_MAX) return ColorClass.PURPLE;

        return ColorClass.UNKNOWN;
    }

    private void mechanumDrive(){
        for (LynxModule hub : hubs) hub.clearBulkCache();

        // Reset yaw if driver taps Y
        if (gamepad1.y) imu.resetYaw();

        // ====== Read & shape inputs ======
        double rawY  = -gamepad1.left_stick_y;   // forward is -Y on gamepad
        double rawX  =  gamepad1.left_stick_x;   // strafe
        double rawRX =  gamepad1.right_stick_x;  // rotate

        double y  = shape(deadband(rawY,  DEADBAND),  INPUT_CURVE_EXP);
        double x  = shape(deadband(rawX,  DEADBAND),  INPUT_CURVE_EXP);
        double rx = shape(deadband(rawRX, DEADBAND),  INPUT_CURVE_EXP);

        // ====== Speed mode toggles ======
        double speedScale = NORMAL_SPEED;
        if (gamepad1.left_bumper)  speedScale = SLOW_SPEED;
        if (gamepad1.right_bumper) speedScale = FAST_SPEED;

        // ====== Field-centric transform ======
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX =  x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY =  x * Math.sin(-heading) + y * Math.cos(-heading);

        // ====== Mecanum mixing (weighted so translation keeps priority) ======
        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + ROTATION_WEIGHT * Math.abs(rx), 1.0);
        double flCmd = (rotY + rotX + rx) / denom;
        double blCmd = (rotY - rotX + rx) / denom;
        double frCmd = (rotY - rotX - rx) / denom;
        double brCmd = (rotY + rotX - rx) / denom;

        // ====== Slew-rate limit (smooth acceleration/braking) ======
        double dt = loopTimer.seconds();
        loopTimer.reset();

        flCmd = slew(flCmd * speedScale, prevFL, MAX_SLEW_PER_SEC, dt);
        frCmd = slew(frCmd * speedScale, prevFR, MAX_SLEW_PER_SEC, dt);
        blCmd = slew(blCmd * speedScale, prevBL, MAX_SLEW_PER_SEC, dt);
        brCmd = slew(brCmd * speedScale, prevBR, MAX_SLEW_PER_SEC, dt);

        prevFL = flCmd; prevFR = frCmd; prevBL = blCmd; prevBR = brCmd;

        // ====== Send to motors ======
        fl.setPower(flCmd);
        fr.setPower(frCmd);
        bl.setPower(blCmd);
        br.setPower(brCmd);

        // ====== Useful telemetry ======
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Speed mode", speedLabel(speedScale));
        telemetry.addData("Cmd", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", flCmd, frCmd, blCmd, brCmd);
        telemetry.update();
    }

    private static double deadband(double v, double dz) {
        return (Math.abs(v) < dz) ? 0.0 : (v - Math.signum(v) * dz) / (1.0 - dz);
    }

    private static double shape(double v, double exp) {
        // keep sign, apply exponential (odd power keeps symmetry). exp=1 => linear, 3=>cubic
        return Math.copySign(Math.pow(Math.abs(v), exp), v);
    }

    private static double slew(double target, double prev, double maxPerSec, double dt) {
        double maxDelta = maxPerSec * dt;
        double delta = target - prev;
        if (Math.abs(delta) > maxDelta) delta = Math.signum(delta) * maxDelta;
        return prev + delta;
    }

    private static String speedLabel(double s) {
        if (Math.abs(s - FAST_SPEED) < 1e-6) return "FAST";
        if (Math.abs(s - SLOW_SPEED) < 1e-6) return "SLOW";
        return "NORMAL";
    }
}

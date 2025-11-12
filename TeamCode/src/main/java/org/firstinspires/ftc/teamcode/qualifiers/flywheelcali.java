package org.firstinspires.ftc.teamcode.qualifiers;


import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@TeleOp(name = "Flywheel Testing (Localization)", group = "qualifiers")
public class flywheelcali extends LinearOpMode {

    private static final String FLYWHEEL_MOTOR = "flywheel";

    private DcMotorEx flywheel;

    private double targetRpm = 0.0;

    boolean startup = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        //init stuff
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            final boolean lb = gamepad1.left_bumper;
            final boolean rb = gamepad1.right_bumper;
            final boolean dpad_up = gamepad1.dpad_up;

            if (dpad_up) {
                flywheel.setPower(.7);
                startup = true;
            }
            if (lb) {
                if (startup){
                    flywheel.setPower(0);
                    startup = false;
                }
                targetRpm += 100;
            }

            if (rb) {
                if (startup){
                    flywheel.setPower(0);
                    startup = false;
                }
                targetRpm = 500;
            }

            if (!startup) {flywheel.setVelocity(targetRpm);}
            telemetry.addLine("Controls: LB = increase speed, RB = reset speed");
            telemetry.update();
        }
    }
}
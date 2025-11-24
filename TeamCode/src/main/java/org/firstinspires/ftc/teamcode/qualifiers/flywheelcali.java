package org.firstinspires.ftc.teamcode.qualifiers;


import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;


@TeleOp(name = "Flywheel Cali", group = "qualifiers")
public class flywheelcali extends LinearOpMode {

    private static final String FLYWHEEL_MOTOR = "flywheel";

    private DcMotorEx flywheel, intake;

    private double targetRpm = 0.0;

    boolean startup = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        //init stuff
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO if you need to reverse the direction uncomment this hunter
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);

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

            if (gamepad1.y){
                intake.setPower(100);
            } else if (gamepad1.a) {
                intake.setPower(0);
            }


            if (!startup) {flywheel.setVelocity(targetRpm);}
            telemetry.addLine("Controls: LB = increase speed, RB = reset speed");
            telemetry.addLine("Target RPM:"+ targetRpm);
            telemetry.update();
        }
    }
}
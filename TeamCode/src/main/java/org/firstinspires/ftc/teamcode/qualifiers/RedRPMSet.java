package org.firstinspires.ftc.teamcode.qualifiers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.LocalizationHelper;
import org.firstinspires.ftc.teamcode.mecanumStuff.MecanumDrive;

public class RedRPMSet extends LinearOpMode {
    qualifiersHardwareMap hardware = new qualifiersHardwareMap();

    // Red alliance goal position (in inches)
    private static final double RED_GOAL_X = 60.0;
    private static final double RED_GOAL_Y = 60.0;

    //Flywheel stuff
    private double targetRpm = 0.0;
    boolean rb_prev = false;
    boolean lb_prev = false;
    int intake_state = -1;
    int flywheel_state = -1;


    DcMotorEx flywheel = hardware.flywheel;
    DcMotorEx intake = hardware.intake;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize localization (this sets up the robot's position tracking)
        MecanumDrive drive = LocalizationHelper.initializeForTeleOp(hardwareMap);
        //Init stuff
        hardware.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            //Controls
            final boolean rb = gamepad1.right_bumper;
            final boolean lb = gamepad1.left_bumper;

            // Update localization and display position (combines update + position telemetry)
            LocalizationHelper.updateWithTelemetry(drive, telemetry, false);

            // Get distance to goal using LocalizationHelper
            double distanceInches = LocalizationHelper.getDistanceToTargetInches(drive, RED_GOAL_X, RED_GOAL_Y);

            // Display distance on telemetry
            telemetry.addLine();
            telemetry.addData("Distance to Red Goal (in)", "%.1f", distanceInches);
            telemetry.addLine();

            //Intake Case
            intake_state = intake_state % 3;
            switch (intake_state) {
                case 0:
                    intake.setPower(75);
                case 1:
                    intake.setPower(-75);
                case 2:
                    intake.setPower(0);
                default:
                    intake.setPower(0);
            }

            //Flywheel Case
            flywheel_state = flywheel_state % 2;
            switch (flywheel_state) {
                case 0:
                    flywheel.setVelocity(targetRpm);
                case 1:
                    flywheel.setVelocity(0);
                default:
                    flywheel.setVelocity(0);
            }


            //Flywheel button - right bumper
            if (rb && !rb_prev) {
                targetRpm = (distanceInches + 144) / .12;
                flywheel_state += 1;
            }

            //Intake button - left bumper
            if (lb && !lb_prev) {
                intake_state += 1;
            }




            // Update previous button states
            lb_prev = lb;
            rb_prev = rb;

            }

            telemetry.update();
        }
    }
}

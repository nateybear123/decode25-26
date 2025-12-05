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


    DcMotorEx flywheel = hardware.flywheel;
    DcMotorEx intake = hardware.intake;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize localization (this sets up the robot's position tracking)
        MecanumDrive drive = LocalizationHelper.initializeForTeleOp(hardwareMap);
        //Init stuff
        hardware.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            //Controls
            final boolean rb = gamepad1.right_bumper;

            // Update localization and display position (combines update + position telemetry)
            LocalizationHelper.updateWithTelemetry(drive, telemetry, false);

            // Get distance to goal using LocalizationHelper
            double distanceInches = LocalizationHelper.getDistanceToTargetInches(drive, RED_GOAL_X, RED_GOAL_Y);

            // Display distance on telemetry
            telemetry.addLine();
            telemetry.addData("Distance to Red Goal (in)", "%.1f", distanceInches);
            telemetry.addLine();

            //TODO Put code here
            // Example: Use distance for logic
            if (rb && !rb_prev) {
                //targetRpm = ((distanceInches - b)/m); //TODO Uncomment line when replaced with values
                //b and m are temp variables for regression (y=mx+b)

                wait(1500);
                //launch first artifact

                wait(1000);
                //launch second artifact

                wait(1000);
                //launch third artifact

                targetRpm = 0;

            }

            telemetry.update();
        }
    }
}

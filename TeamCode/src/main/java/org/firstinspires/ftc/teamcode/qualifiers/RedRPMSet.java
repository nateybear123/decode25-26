package org.firstinspires.ftc.teamcode.qualifiers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LocalizationHelper;
import org.firstinspires.ftc.teamcode.mecanumStuff.MecanumDrive;

public class RedRPMSet extends LinearOpMode {
    // Blue alliance goal position (in inches)
    private static final double BLUE_GOAL_X = 60.0;
    private static final double BLUE_GOAL_Y = 60.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize localization (this sets up the robot's position tracking)
        MecanumDrive drive = LocalizationHelper.initializeForTeleOp(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            // Update localization and display position (combines update + position telemetry)
            LocalizationHelper.updateWithTelemetry(drive, telemetry, false);

            // Get distance to goal using LocalizationHelper
            double distanceMeters = LocalizationHelper.getDistanceToTargetMeters(drive, BLUE_GOAL_X, BLUE_GOAL_Y);
            double distanceInches = LocalizationHelper.getDistanceToTargetInches(drive, BLUE_GOAL_X, BLUE_GOAL_Y);

            // Display distance on telemetry
            telemetry.addLine();
            telemetry.addData("Distance to Blue Goal (m)", "%.2f", distanceMeters);
            telemetry.addData("Distance to Blue Goal (in)", "%.1f", distanceInches);
            telemetry.addLine();

            //TODO Put code here
            // Example: Use distance for logic
            if (distanceMeters < 1.5) {
                telemetry.addLine("Within shooting range!");
            } else {
                telemetry.addLine("Too far from goal");
            }

            telemetry.update();
        }
    }
}

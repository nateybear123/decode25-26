package org.firstinspires.ftc.teamcode.qualifiers;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LocalizationHelper;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.mecanumStuff.MecanumDrive;

@Autonomous(name = "Close Red Auto", group = "qualifiers")
public class CloseRedAuto extends LinearOpMode {

    // Starting position for Close Red
    static double startX = -50;
    static double startY = 50;
    static double startHeading = -45; // In degrees
    static Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));

    // Launch position (where to shoot artifacts)
    // TODO: Verify this position on your actual field
    static double launchX = -16;
    static double launchY = 0;
    static Vector2d launchPose = new Vector2d(launchX, launchY);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize localization for autonomous
        MecanumDrive drive = LocalizationHelper.initializeForAuto(hardwareMap, startPose);

        // TODO: Initialize your robot hardware (servos, motors, etc.)

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "Red");
        telemetry.addData("Position", "Close");
        telemetry.addData("Starting Pose", "X: %.2f, Y: %.2f, Heading: %.1f°",
            startPose.position.x,
            startPose.position.y,
            Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Run autonomous path
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                .lineToX(launchX)
                // TODO: Add your shooting mechanism here
                .waitSeconds(4) // Placeholder for shooting
                .strafeToLinearHeading(new Vector2d(-12, 32), Math.toRadians(-270))
                // TODO: Add specimen pickup here
                .strafeToLinearHeading(new Vector2d(-12, 52), Math.toRadians(-270))
                .strafeToLinearHeading(new Vector2d(-12, 40), Math.toRadians(-270))
                .splineTo(launchPose, Math.toRadians(135))
                .waitSeconds(4) // Placeholder for shooting
                .strafeToLinearHeading(new Vector2d(12, 32), Math.toRadians(-270))
                // TODO: Add specimen pickup here
                .strafeToLinearHeading(new Vector2d(12, 52), Math.toRadians(-270))
                .strafeToLinearHeading(new Vector2d(12, 40), Math.toRadians(-270))
                .splineTo(launchPose, Math.toRadians(135))
                .waitSeconds(4) // Placeholder for shooting
                .build()
        );

        // CRITICAL: Save final pose for TeleOp
        LocalizationHelper.savePoseForTeleOp(drive);

        telemetry.addData("Status", "Auto Complete");
        telemetry.addData("Final Pose", "X: %.2f, Y: %.2f, Heading: %.1f°",
            PoseStorage.currentPose.position.x,
            PoseStorage.currentPose.position.y,
            Math.toDegrees(PoseStorage.currentPose.heading.toDouble()));
        telemetry.update();

        sleep(1000);
    }
}

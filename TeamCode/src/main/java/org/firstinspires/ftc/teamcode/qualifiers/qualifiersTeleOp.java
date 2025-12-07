package org.firstinspires.ftc.teamcode.qualifiers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LocalizationHelper;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.mecanumStuff.MecanumDrive;


@TeleOp(name = "TeleOp", group = "qualifiers")
public class qualifiersTeleOp extends LinearOpMode {

    qualifiersHardwareMap hardware = new qualifiersHardwareMap();
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hardware.init(hardwareMap);

        // Initialize Road Runner drive with starting pose from autonomous
        // If no auto was run, defaults to (0, 0, 0)
        drive = LocalizationHelper.initializeForTeleOp(hardwareMap);

        telemetry.addData("Starting Position", "X: %.2f, Y: %.2f, Heading: %.1f°",
            PoseStorage.currentPose.position.x,
            PoseStorage.currentPose.position.y,
            Math.toDegrees(PoseStorage.currentPose.heading.toDouble()));
        telemetry.addLine();
        telemetry.addLine("=== MANUAL POSITION OVERRIDE ===");
        telemetry.addLine("A: Far Red (56, -8, -225°)");
        telemetry.addLine("B: Close Red (-50, 50, -45°)");
        telemetry.addLine("X: Far Blue (56, -8, -225°)");
        telemetry.addLine("Y: Close Blue (-50, -50, 45°)");
        telemetry.addLine("DPAD_DOWN: Reset to (0, 0, 0)");
        telemetry.addLine();
        telemetry.addData("Status", "Ready - Press buttons to override position");
        telemetry.update();

        // Manual position selection during init
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                // Far Red position
                LocalizationHelper.resetPosition(drive, new Pose2d(56, -8, Math.toRadians(-225)));
                telemetry.addData("Position Override", "Far Red (56, -8, -225°)");
                telemetry.update();
                sleep(300); // Debounce
            }
            else if (gamepad1.b) {
                // Close Red position
                LocalizationHelper.resetPosition(drive, new Pose2d(-50, 50, Math.toRadians(-45)));
                telemetry.addData("Position Override", "Close Red (-50, 50, -45°)");
                telemetry.update();
                sleep(300);
            }
            else if (gamepad1.x) {
                // Far Blue position
                LocalizationHelper.resetPosition(drive, new Pose2d(56, -8, Math.toRadians(-225)));
                telemetry.addData("Position Override", "Far Blue (56, -8, -225°)");
                telemetry.update();
                sleep(300);
            }
            else if (gamepad1.y) {
                // Close Blue position
                LocalizationHelper.resetPosition(drive, new Pose2d(-50, -50, Math.toRadians(45)));
                telemetry.addData("Position Override", "Close Blue (-50, -50, 45°)");
                telemetry.update();
                sleep(300);
            }
            else if (gamepad1.dpad_down) {
                // Reset to origin
                LocalizationHelper.resetPosition(drive, new Pose2d(0, 0, 0));
                telemetry.addData("Position Override", "Origin (0, 0, 0)");
                telemetry.update();
                sleep(300);
            }
        }


        //TODO UPDATE MOVING CODE TO BE BETTER (DRIFTING AND WHAT NOT)
        while (opModeIsActive()){

            // Update localization - THIS IS CRITICAL!
            LocalizationHelper.update(drive);

            // Get current pose
            Pose2d currentPose = LocalizationHelper.getCurrentPose(drive);

            // Display position on telemetry
            telemetry.addData("X Position", currentPose.position.x);
            telemetry.addData("Y Position", currentPose.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.update();

            // Movement
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            hardware.fr.setPower(frontRightPower);
            hardware.br.setPower(backRightPower); //change to reverse in hw map
            hardware.fl.setPower(frontLeftPower);
            hardware.bl.setPower(backLeftPower);
        }
    }


    //TODO OLD COLOR DETECTION (DUNNO IF USING HUSKYCAM OR ACTUAL CAM) SO FIGURE THAT OUT
//    public void colorRecognition(){
//        HuskyLens.Block[] blocks = hardware.huskyLens.blocks();
//        telemetry.addData("Block Count", blocks.length);
//        for (int i = 0; i < blocks.length; i++) {
//            int thisColorID = blocks[i].id;
//
//            switch(thisColorID){
//                case 1:
//                    //Code for if it's Blue (ID = 1)
//                    telemetry.addData("Color ID","Blue");
//                    blinkinBlue();
//                    gamepad1.setLedColor(0,0,255,1000);
//                    gamepad2.setLedColor(0,0,255,1000);
//                    break;
//                case 2:
//                    //SET UP RED AS ID = 2
//                    //Code for if it's Red (ID = 2)
//                    telemetry.addData("Color ID", "Red");
//                    blinkinRed();
//                    gamepad1.setLedColor(255,0,0,1000);
//                    gamepad2.setLedColor(255,0,0,1000);
//                    break;
//                case 3:
//                    //SET UP YELLOW AS ID = 3
//                    //Code for if it's Yellow (ID = 3)
//                    telemetry.addData("Color ID","Yellow");
//                    blinkinYellow();
//                    gamepad1.setLedColor(225,225,0,1000);
//                    gamepad2.setLedColor(225,225,0,1000);
//                    break;
//                default:
//                    blinkinBlack();
//                    gamepad1.setLedColor(113,202,235,1000);
//                    gamepad2.setLedColor(113,202,235,1000);
//            }
//        }
//    }

//    public void blinkinRed(){
//        hardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//    }
//    public void blinkinGreen(){hardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);}
//    public void blinkinYellow(){hardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);}
//    public void blinkinBlue(){hardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);}
//    public void blinkinBlack(){hardware.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);}
}

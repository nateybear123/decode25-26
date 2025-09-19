package org.firstinspires.ftc.teamcode.talhaOutreachBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Talha Outreach Bot", group="TeleOp")
public class talhaOutreachBot extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Mapping the motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Reverse left motors
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press start
        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double demominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower  = (y + x + rx) / demominator;
            double backLeftPower   = (y - x + rx) / demominator;
            double frontRightPower = (y - x - rx) / demominator;
            double backRightPower  = (y + x - rx) / demominator;

            // Set motor powers
            frontLeft.setPower(frontLeftPower / 1.4);
            backLeft.setPower(backLeftPower / 1.4);
            frontRight.setPower(frontRightPower / 1.4);
            backRight.setPower(backRightPower / 1.4);

            

            // Telemetry
            telemetry.addData("FL", frontLeftPower);
            telemetry.addData("FR", frontRightPower);
            telemetry.addData("BL", backLeftPower);
            telemetry.addData("BR", backRightPower);
            telemetry.update();
        }
    }
}

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
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        // Reverse left motors
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press start
        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            double forward = gamepad1.right_trigger;
            double backward = gamepad1.left_trigger;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double y = forward - backward;
            double s = strafe * 1.1;
            double r = rotate;

            double frontLeftPower  = y + s + r;
            double backLeftPower   = y - s + r;
            double frontRightPower = y - s - r;
            double backRightPower  = y + s - r;

            // Normalize powers
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                backLeftPower   /= max;
                frontRightPower /= max;
                backRightPower  /= max;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Telemetry
            telemetry.addData("FL", frontLeftPower);
            telemetry.addData("FR", frontRightPower);
            telemetry.addData("BL", backLeftPower);
            telemetry.addData("BR", backRightPower);
            telemetry.update();
        }
    }
}

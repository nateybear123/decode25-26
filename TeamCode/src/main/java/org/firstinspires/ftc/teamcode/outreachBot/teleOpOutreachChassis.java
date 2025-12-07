package org.firstinspires.ftc.teamcode.outreachBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Outreach TeleOp", group = "Outreach")

public class teleOpOutreachChassis extends LinearOpMode {

    DcMotorEx backLeftDrive, backRightDrive, frontLeftDrive, frontRightDrive;

    DcMotorEx armLeft, armRight;

    Servo claw;

    float armPower;


    @Override
    public void runOpMode() throws InterruptedException {

        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotorEx.class,"frontRight");
        frontLeftDrive = hardwareMap.get(DcMotorEx.class,"frontLeft");

        armLeft = hardwareMap.get(DcMotorEx.class,"armLeft");
        armRight = hardwareMap.get(DcMotorEx.class,"armRight");

        claw = hardwareMap.get(Servo.class,"claw");

        frontRightDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()){

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontRightDrive.setPower(frontRightPower / 1.4);
            backRightDrive.setPower(backRightPower / 1.4); //change to reverse in hw map
            frontLeftDrive.setPower(frontLeftPower / 1.4);
            backLeftDrive.setPower(backLeftPower / 1.4);

            //Open Claw
            if(gamepad1.left_trigger>0){
                claw.setPosition(0.3);
            }

            //Close Claw
            if(gamepad1.right_trigger>0){
                claw.setPosition(0);
            }

            //Arm Down and Up
            if(gamepad1.left_bumper){
                armPower = -0.7F;
                armLeft.setPower(-armPower);
                armRight.setPower(armPower);
            } else if(gamepad1.right_bumper) {
                armPower = 0.5F;
                armLeft.setPower(-armPower);
                armRight.setPower(armPower);
            } else {
                armLeft.setPower(0);
                armRight.setPower(0);
            }




            //Telemetry

            telemetry.addData("Back Left Velocity",backLeftDrive.getVelocity());
            telemetry.addData("Back Left Power",backLeftDrive.getPower());

            telemetry.addData("Back Right Velocity",backRightDrive.getVelocity());
            telemetry.addData("Back Right Power",backRightDrive.getPower());

            telemetry.addData("Front Left Velocity",frontLeftDrive.getVelocity());
            telemetry.addData("Front Left Power",frontLeftDrive.getPower());

            telemetry.addData("Front Right Velocity",frontRightDrive.getVelocity());
            telemetry.addData("Front Right Power",frontRightDrive.getPower());

            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.hunterOutreachBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hunter Outreach TeleOp", group = "Outreach")

public class hunterOutreachBot extends LinearOpMode {

    DcMotorEx bld, brd, fld, frd;

    DcMotorEx Larm, Rarm;

    Servo claw;

    float aPwr;


    @Override
    public void runOpMode() throws InterruptedException {

        brd = hardwareMap.get(DcMotorEx.class, "backRight");
        bld = hardwareMap.get(DcMotorEx.class, "backLeft");
        frd = hardwareMap.get(DcMotorEx.class,"frontRight");
        fld = hardwareMap.get(DcMotorEx.class,"frontLeft");

        Larm = hardwareMap.get(DcMotorEx.class,"armLeft");
        Rarm = hardwareMap.get(DcMotorEx.class,"armRight");

        claw = hardwareMap.get(Servo.class,"claw");

        frd.setDirection(DcMotorEx.Direction.REVERSE);
        brd.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        brd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bld.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fld.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Larm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()){

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frd.setPower(frontRightPower / 1.4);
            brd.setPower(backRightPower / 1.4);
            fld.setPower(frontLeftPower / 1.4);
            bld.setPower(backLeftPower / 1.4);

            //Open
            if(gamepad1.left_trigger>0){
                claw.setPosition(0.3);
            }

            //Close
            if(gamepad1.right_trigger>0){
                claw.setPosition(0);
            }

            //Arm movement
            if(gamepad1.left_bumper){
                aPwr = -0.7F;
                Larm.setPower(-aPwr);
                Rarm.setPower(aPwr);
            } else if(gamepad1.right_bumper) {
                aPwr = 0.5F;
                Larm.setPower(-aPwr);
                Rarm.setPower(aPwr);
            } else {
                Larm.setPower(0);
                Rarm.setPower(0);
            }





            telemetry.addData("Back Left Velocity",bld.getVelocity());
            telemetry.addData("Back Left Power",bld.getPower());

            telemetry.addData("Back Right Velocity",brd.getVelocity());
            telemetry.addData("Back Right Power",brd.getPower());

            telemetry.addData("Front Left Velocity",fld.getVelocity());
            telemetry.addData("Front Left Power",fld.getPower());

            telemetry.addData("Front Right Velocity",frd.getVelocity());
            telemetry.addData("Front Right Power",frd.getPower());

            telemetry.update();
        }
    }
}

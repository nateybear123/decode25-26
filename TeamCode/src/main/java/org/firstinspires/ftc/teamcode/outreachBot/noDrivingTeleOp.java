package org.firstinspires.ftc.teamcode.outreachBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "NO DRIVING Outreach Teleop", group = "Outreach")

public class noDrivingTeleOp extends LinearOpMode {


    DcMotorEx armLeft, armRight;

    Servo claw;

    float armPower;

    boolean clawClosed;


    @Override
    public void runOpMode() throws InterruptedException {



        armLeft = hardwareMap.get(DcMotorEx.class,"armLeft");
        armRight = hardwareMap.get(DcMotorEx.class,"armRight");

        claw = hardwareMap.get(Servo.class,"claw");



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()){


            //Open Claw
            if(gamepad1.left_trigger>0){
                clawClosed = false;
                claw.setPosition(0.3);
            }

            //Close Claw
            if(gamepad1.right_trigger>0){
                clawClosed = true;
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

           telemetry.addData("Claw Closed? ", clawClosed);
            telemetry.addData("Arm Position: ", armLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}

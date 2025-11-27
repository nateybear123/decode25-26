package org.firstinspires.ftc.teamcode.qualifiers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Test", group = "qualifiers")
public class encoderTest extends LinearOpMode {

    qualifiersHardwareMap hardware = new qualifiersHardwareMap();

    int motor = 1;
    String motorSelected;


    @Override
    public void runOpMode() throws InterruptedException {

        hardware.init(hardwareMap);
        hardware.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardware.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardware.fl.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.fr.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.bl.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.br.setDirection(DcMotorSimple.Direction.FORWARD);



        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                if (motor < 4) {
                    motor++;
                } else {
                    motor = 1;
                }
            }
            // Movement
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed

            hardware.fr.setPower(y);
            hardware.br.setPower(y); //change to reverse in hw map
            hardware.fl.setPower(y);
            hardware.bl.setPower(y);

            switch (motor) {
                case 1:
                    motorSelected = "Front Left";
                    break;
                case 2:
                    motorSelected = "Front Right";
                    break;
                case 3:
                    motorSelected = "Back Left";
                    break;
                case 4:
                    motorSelected = "Back Right";
                    break;
            }
            telemetry.addData("Motor Selected:", motorSelected);
            telemetry.addData("FL Position", hardware.fl.getCurrentPosition());
            telemetry.addData("FR Position", hardware.fr.getCurrentPosition());
            telemetry.addData("BL Position", hardware.bl.getCurrentPosition());
            telemetry.addData("BR Position", hardware.br.getCurrentPosition());

            telemetry.update();
        }
    }
}

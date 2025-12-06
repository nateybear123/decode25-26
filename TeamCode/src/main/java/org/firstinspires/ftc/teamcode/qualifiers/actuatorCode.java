package org.firstinspires.ftc.teamcode.qualifiers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class actuatorCode extends LinearOpMode {

    private DcMotor leftActuator;
    private DcMotor rightActuator;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize actuators
        leftActuator = hardwareMap.get(DcMotor.class, "leftActuator");
        rightActuator = hardwareMap.get(DcMotor.class, "rightActuator");

        leftActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()){

            // Control actuators with dpad
            if (gamepad1.dpad_up) {
                // Extend actuators
                leftActuator.setPower(1.0);
                rightActuator.setPower(1.0);
            } else if (gamepad1.dpad_down) {
                // Retract actuators
                leftActuator.setPower(-1.0);
                rightActuator.setPower(-1.0);
            } else {
                // Stop actuators when no button is pressed
                leftActuator.setPower(0);
                rightActuator.setPower(0);
            }
        }
    }
}

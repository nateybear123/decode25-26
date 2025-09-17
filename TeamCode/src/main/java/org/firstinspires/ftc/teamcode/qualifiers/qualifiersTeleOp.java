package org.firstinspires.ftc.teamcode.qualifiers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "TeleOp")
public class qualifiersTeleOp extends LinearOpMode {

    int linearMin = 0;
    double clawPos = 0.35;

    qualifiersHardwareMap hardware = new qualifiersHardwareMap();

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hardware.init(hardwareMap);



        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();


        //TODO UPDATE MOVING CODE TO BE BETTER (DRIFTING AND WHAT NOT)
        while (opModeIsActive()){

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

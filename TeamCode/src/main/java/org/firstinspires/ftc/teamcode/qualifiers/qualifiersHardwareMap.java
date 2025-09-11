package org.firstinspires.ftc.teamcode.qualifiers;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class qualifiersHardwareMap {
    HuskyLens huskyLens;

    // Regular Movement DC Motors
    DcMotorEx bl, br, fl, fr;

    NormalizedColorSensor colorSensor;

    HardwareMap hardwareMap;


    // Sensors
    public IMU imu;

    public void init(HardwareMap ahwMap){
        hardwareMap = ahwMap;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        //Camera
        huskyLens = hardwareMap.get(HuskyLens.class,"huskyLens");
//

        // Initialization Code
        br = hardwareMap.get(DcMotorEx.class, "backRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        fr = hardwareMap.get(DcMotorEx.class,"frontRight");
        fl = hardwareMap.get(DcMotorEx.class,"frontLeft");


        imu = hardwareMap.get(IMU.class,"imu");


        // Movement (might not be necessary/waste power)
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }


}
package org.firstinspires.ftc.teamcode.qualifiers;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

public class qualifiersHardwareMap {
    public DcMotorEx bl, br, fl, fr;
//    public NormalizedColorSensor colorSensor;
    public IMU imu;

//    HuskyLens huskyLens;

    public void init(HardwareMap hardwareMap){
        // Color Sensor
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        // Camera
//        huskyLens  = hardwareMap.get(HuskyLens.class,"huskyLens");

        br = hardwareMap.get(DcMotorEx.class, "backRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");

        imu = hardwareMap.get(IMU.class, "imu");

        // Zero power behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO FIGURE OUT WHICH NEEDS TO BE REVERSE OR FORWARD
//         fl.setDirection(DcMotorSimple.Direction.REVERSE);
//         bl.setDirection(DcMotorSimple.Direction.REVERSE);
//         fr.setDirection(DcMotorSimple.Direction.FORWARD);
//         br.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
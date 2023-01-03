package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class TwoMRevDistanceSensor extends OpMode{
    DistanceSensor dsensor;
    @Override
    public void init() {

        dsensor = hardwareMap.get(DistanceSensor.class, "distance sensor")


    }
    public void distance() {
        double value = dsensor.getDistance(DistanceUnit.INCH);
        if (value > 0){
            telemetry.addData("Distance: ", value);
            // add code to move robot

        }
    }
    @Override
    public void loop(){

    }

    @Override
    public void stop() {

    }

}

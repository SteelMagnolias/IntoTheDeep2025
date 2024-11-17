package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name= "sensorTesting", group="Iterative Opmode")
public class sensorTesting extends OpMode {
    // declare motors
    private DcMotor armLeft;
    private DcMotor armEncoder;

    // sensors
    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;

    public void init () {
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");

        //encoder setup
        armEncoder = armLeft;

        // sensors
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");

        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");

        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        telemetry.addData("arm encoder", armEncoder.getCurrentPosition());

        telemetry.addData("colorLeft Red:", colorLeft.red());
        telemetry.addData("colorLeft Blue:", colorLeft.blue());
        telemetry.addData("colorLeft Green: ", colorLeft.green());

        telemetry.addData("Color Sensor colorRight Red:", colorRight.red());
        telemetry.addData("Color Sensor colorRight Blue:", colorRight.blue());
        telemetry.addData("Color Sensor colorRight Green: ", colorRight.green());

        telemetry.addData("distanceLeft:", distanceLeft.getDistance(DistanceUnit.INCH));
        telemetry.addData("distanceRight:", distanceRight.getDistance(DistanceUnit.INCH));

        telemetry.update();

    }
}
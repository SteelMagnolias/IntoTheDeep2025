package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name= "colorTesting", group="Iterative Opmode")
public class colorTesting extends OpMode {
    // sensors
    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    public void init () {
        // sensors
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
    }

    @Override
    public void loop() {

        telemetry.addData("colorLeft Red:", colorLeft.red());
        telemetry.addData("colorLeft Blue:", colorLeft.blue());
        telemetry.addData("colorLeft Green: ", colorLeft.green());

        telemetry.addData("Color Sensor colorRight Red:", colorRight.red());
        telemetry.addData("Color Sensor colorRight Blue:", colorRight.blue());
        telemetry.addData("Color Sensor colorRight Green: ", colorRight.green());

        telemetry.update();

    }
}
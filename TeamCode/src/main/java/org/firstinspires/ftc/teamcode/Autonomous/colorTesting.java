package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Disabled
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

        telemetry.addData("Color Left Red:", colorLeft.red());
        telemetry.addData("Color Left Blue:", colorLeft.blue());
        telemetry.addData("Color Left Green: ", colorLeft.green());

        telemetry.addData("Color Right Red:", colorRight.red());
        telemetry.addData("Color Right Blue:", colorRight.blue());
        telemetry.addData("Color Right Green: ", colorRight.green());

        telemetry.update();

    }
}
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "linearSlidesTesting" , group = "Iterative Opmode")
public class linearSlidesTesting extends OpMode {

    private DcMotor motor;

    double pow = 0.9;

    public void init() {
        motor  = hardwareMap.get(DcMotor.class, "motor");

    }

    public void loop() {
        double lefty1 = -(gamepad1.left_stick_y);

        if (Math.abs(lefty1) > 0.1){
            motor.setPower(lefty1 * pow);
        } else {
            motor.setPower(0);
        }
    }
}
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "linearSlidesPIDTuning", group = "Iterative Opmode")
public class linearSlidesPIDTuning extends OpMode {

    private DcMotor linearSlidesLeft;
    private DcMotor armEncoder;

    ElapsedTime armTimer = new ElapsedTime();

    double linearSlidesPow;
    double desLength;
    double ArmLength;
    double currentTime;
    double previousTime;
    double currentError;
    double previousError;
    double P;
    double I;
    double D;
    double SP = 0.00535;
    double SI = 0.000002;
    double SD = 0.5;

    public void init() {
        linearSlidesLeft = hardwareMap.get(DcMotor.class, "linearSlidesLeft");

        //encoder setup
        armEncoder = linearSlidesLeft;

        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        double lefty2 = -(gamepad2.left_stick_y); // this is the value of gamepad2's left joystick y value
        boolean a2 = gamepad2.a; // this is the value of the a button on gamepad2

        desLength += lefty2 * 3;

        if(a2) {
            //PID stuff
            ArmLength = -armEncoder.getCurrentPosition();
            currentError = ArmLength - desLength;
            currentTime = armTimer.milliseconds();

            P = currentError * SP;
            I = SI * (currentError * (currentTime - previousTime));
            D = SD * (currentError - previousError) / (currentTime - previousTime);
            linearSlidesPow = (P + I + D);

            previousTime = currentTime;
            previousError = currentError;

            linearSlidesLeft.setPower(linearSlidesPow);
        }

        telemetry.addData("linearSlidesPow", linearSlidesPow);
        telemetry.addData("desired length", desLength);
        telemetry.addData("length", ArmLength);
        telemetry.update();
    }
}

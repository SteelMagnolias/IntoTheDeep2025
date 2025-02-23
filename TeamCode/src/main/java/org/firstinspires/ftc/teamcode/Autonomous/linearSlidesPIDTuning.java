package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "linearSlidesPIDTuning", group = "Iterative Opmode")
public class linearSlidesPIDTuning extends OpMode {

    private DcMotor armSlide;
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
    double SP = 0.002;
    double SI = 0.00001;
    double SD = 0.05;

    public void init() {
        armSlide = hardwareMap.get(DcMotor.class, "armSlide");

        //encoder setup
        armEncoder = armSlide;

        armSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double lefty2 = -(gamepad2.left_stick_y); // this is the value of gamepad2's left joystick y value
        boolean a2 = gamepad2.a; // this is the value of the a button on gamepad2
        boolean b2 = gamepad2.b;

        desLength += lefty2 * 8;

        //PID stuff
        ArmLength = armEncoder.getCurrentPosition();
        currentError = ArmLength - desLength;
        currentTime = armTimer.milliseconds();

        P = currentError * SP;
        I = SI * (currentError * (currentTime - previousTime));
        D = SD * (currentError - previousError) / (currentTime - previousTime);
        linearSlidesPow = (P + I + D);
        if(linearSlidesPow < -0.7) linearSlidesPow = -0.7;
        if(linearSlidesPow > 0.7) linearSlidesPow = 0.7;

        previousTime = currentTime;
        previousError = currentError;


        if(a2) {
            armSlide.setPower(linearSlidesPow);
            telemetry.addLine("running motor");
        } else if (b2) {
            previousError = 0;
            previousTime = 0;
            armTimer.reset();
        } else {
            armSlide.setPower(0);
        }

        telemetry.addData("linearSlidesPow", linearSlidesPow);
        telemetry.addData("desired length", desLength);
        telemetry.addData("length", ArmLength);
        telemetry.update();
    }
}

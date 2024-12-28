package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "armPIDTuning", group = "Iterative Opmode")
public class armPIDTuning extends OpMode {

    private DcMotor armLeft;
    private DcMotor armRight;
    private DcMotor armEncoder;

    ElapsedTime armTimer = new ElapsedTime();

    double armPow;
    double desArmPos;
    double armPos;
    double currentTime;
    double previousTime;
    double currentError;
    double previousError;
    double P;
    double I;
    double D;
    double KP = 0.00535;
    double KI = 0.000002;
    double KD = 0.5;

    public void init() {
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");

        //encoder setup
        armEncoder = armLeft;

        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        double lefty2 = -(gamepad2.left_stick_y); // this is the value of gamepad2's left joystick y value
        boolean a2 = gamepad2.a; // this is the value of the a button on gamepad2

        desArmPos += lefty2 * 3;

        if(a2) {
            //PID stuff
            armPos = -armEncoder.getCurrentPosition();
            currentError = armPos - desArmPos;
            currentTime = armTimer.milliseconds();

            P = currentError * KP;
            I = KI * (currentError * (currentTime - previousTime));
            D = KD * (currentError - previousError) / (currentTime - previousTime);
            armPow = (P + I + D);

            previousTime = currentTime;
            previousError = currentError;

            armLeft.setPower(armPow);
            armRight.setPower(armPow);
        }

        telemetry.addData("armPow", armPow);
        telemetry.addData("desired arm position", desArmPos);
        telemetry.addData("arm position", armPos);
        telemetry.update();
    }
}

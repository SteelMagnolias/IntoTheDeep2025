package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "distanceDrivePIDTuning", group = "Iterative Opmode")
public class distanceDrivePIDTuning extends OpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;

    ElapsedTime PIDTimer = new ElapsedTime();

    double powLeft;
    double powRight;
    double desDis;
    double disLeft;
    double disRight;
    double currentTime;
    double previousTime;
    double currentErrorLeft;
    double previousErrorLeft;
    double currentErrorRight;
    double previousErrorRight;
    double P;
    double I;
    double D;
    double KP = 0.08;
    double KI = 0;
    double KD = 0;

    public void init() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double lefty2 = -(gamepad2.left_stick_y);
        boolean a2 = gamepad2.a;

        desDis += lefty2 * 3;

        if(a2) {
            //PID stuff left
            disLeft = distanceLeft.getDistance(DistanceUnit.CM);
            currentErrorLeft = disLeft - desDis;
            currentTime = PIDTimer.milliseconds();

            P = currentErrorLeft * KP;
            I = KI * (currentErrorLeft * (currentTime - previousTime));
            D = KD * (currentErrorLeft - previousErrorLeft) / (currentTime - previousTime);
            powLeft = (P + I + D);

            previousErrorLeft = currentErrorLeft;

            //PID stuff right
            disRight = distanceRight.getDistance(DistanceUnit.CM);
            currentErrorRight = disRight - desDis;
            currentTime = PIDTimer.milliseconds();

            P = currentErrorRight * KP;
            I = KI * (currentErrorRight * (currentTime - previousTime));
            D = KD * (currentErrorRight - previousErrorRight) / (currentTime - previousTime);
            powRight = (P + I + D);

            previousTime = currentTime;
            previousErrorRight = currentErrorRight;

            rightBack.setPower(powRight);
            rightFront.setPower(powRight);
            leftBack.setPower(powLeft);
            leftFront.setPower(powLeft);
        }

        telemetry.addData("powRight", powRight);
        telemetry.addData("powLeft", powLeft);
        telemetry.addData("desired distance", desDis);
        telemetry.addData("distance Right", disRight);
        telemetry.addData("distance Left", disLeft);
        telemetry.update();
    }
}

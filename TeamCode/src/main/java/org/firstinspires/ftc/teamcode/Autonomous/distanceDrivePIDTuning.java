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
    double DP = 0.025;
    double DI = 0;
    double DD = 0;

    public void init() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double lefty2 = -(gamepad2.left_stick_y);
        boolean a2 = gamepad2.a;

        desDis += lefty2 / 4;

        //PID stuff left
        disLeft = distanceLeft.getDistance(DistanceUnit.CM);
        currentErrorLeft = disLeft - desDis;
        currentTime = PIDTimer.milliseconds();

        P = currentErrorLeft * DP;
        I = DI * (currentErrorLeft * (currentTime - previousTime));
        D = DD * (currentErrorLeft - previousErrorLeft) / (currentTime - previousTime);
        powLeft = (P + I + D);

        previousErrorLeft = currentErrorLeft;

        //PID stuff right
        disRight = distanceRight.getDistance(DistanceUnit.CM);
        currentErrorRight = disRight - desDis;
        currentTime = PIDTimer.milliseconds();

        P = currentErrorRight * DP;
        I = DI * (currentErrorRight * (currentTime - previousTime));
        D = DD * (currentErrorRight - previousErrorRight) / (currentTime - previousTime);
        powRight = (P + I + D);

        previousTime = currentTime;
        previousErrorRight = currentErrorRight;

        if(a2 && disRight < 150 && disLeft < 150 ) {
            rightBack.setPower(powRight);
            rightFront.setPower(powRight);
            leftBack.setPower(powLeft);
            leftFront.setPower(powLeft);
        } else {
            rightBack.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            leftFront.setPower(0);
        }

        telemetry.addData("powRight", powRight);
        telemetry.addData("powLeft", powLeft);
        telemetry.addData("desired distance", desDis);
        telemetry.addData("distance Right", disRight);
        telemetry.addData("distance Left", disLeft);
        telemetry.update();
    }
}

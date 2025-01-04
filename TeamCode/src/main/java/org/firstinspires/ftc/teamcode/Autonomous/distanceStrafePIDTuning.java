package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "distanceStrafePIDTuning", group = "Iterative Opmode")
public class distanceStrafePIDTuning extends OpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DistanceSensor distanceSide;

    ElapsedTime PIDTimer = new ElapsedTime();

    double pow;
    double desDis;
    double disSide;
    double currentTime;
    double previousTime;
    double currentErrorSide;
    double previousErrorSide;
    double P;
    double I;
    double D;
    double DSP = 0.08;
    double DSI = 0;
    double DSD = 0;

    public void init() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        distanceSide = hardwareMap.get(DistanceSensor.class, "distanceLeft");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double lefty2 = -(gamepad2.left_stick_y);
        boolean a2 = gamepad2.a;

        desDis += lefty2 * 3;

        if(a2) {
            //PID stuff
            disSide = distanceSide.getDistance(DistanceUnit.CM);
            currentErrorSide = disSide - desDis;
            currentTime = PIDTimer.milliseconds();

            P = currentErrorSide * DSP;
            I = DSI * (currentErrorSide * (currentTime - previousTime));
            D = DSD * (currentErrorSide - previousErrorSide) / (currentTime - previousTime);
            pow = (P + I + D);

            previousTime = currentTime;
            previousErrorSide = currentErrorSide;

            rightBack.setPower(pow);
            rightFront.setPower(-pow);
            leftBack.setPower(-pow);
            leftFront.setPower(pow);
        }

        telemetry.addData("pow", pow);
        telemetry.addData("desired distance", desDis);
        telemetry.addData("distance", disSide);
        telemetry.update();
    }
}

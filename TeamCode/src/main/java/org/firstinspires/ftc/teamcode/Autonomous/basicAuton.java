package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "basicAuton" , group = "Linear OpMode")
public class basicAuton extends LinearOpMode {
    // declare motors and servos
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // sensors
    private ColorSensor colorLeft;

    // variables
    double pow = -0.2;
    int targetRed = 2000;
    int targetBlue = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        //map motors from configuration to motor names in code
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        //Reverse Motor
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // sensors
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");

        telemetry.addData("colorLeft Red:", colorLeft.red());
        telemetry.addData("colorLeft Blue:", colorLeft.blue());

        telemetry.addData("target red", targetRed);
        telemetry.addData("target blue", targetBlue);

        telemetry.update();

        waitForStart();

        targetBlue = colorLeft.blue() + 1000;
        targetRed = colorLeft.red() + 1000;

        while (colorLeft.red() < targetRed && colorLeft.blue() < targetBlue){ //drive back till we see blue line then stop
            leftFront.setPower(pow);
            leftBack.setPower(pow);
            rightBack.setPower(pow);
            rightFront.setPower(pow);

            telemetry.addData("colorLeft Red:", colorLeft.red());
            telemetry.addData("colorLeft Blue:", colorLeft.blue());

            telemetry.addData("target red", targetRed);
            telemetry.addData("target blue", targetBlue);

            telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }
}
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "groundScoreAuton" , group = "Linear OpMode")
public class groundScoreAuton extends LinearOpMode {
    // declare motors and servos
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private CRServo intake;

    // sensors
    private ColorSensor colorLeft;

    // variables
    double pow = 0.15;
    int targetRed = 2000;
    int targetBlue = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        //map motors from configuration to motor names in code
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intake = hardwareMap.get(CRServo.class, "intake");

        //Reverse Motor
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // sensors
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");

        targetBlue = colorLeft.blue() + 750;
        targetRed = colorLeft.red() + 500;

        waitForStart();

        colorDrive(-1);

        intake.setPower(1);
        sleep(2500);
        intake.setPower(0);

        colorDrive(1);
    }

    public void colorDrive (int dir){
        while (colorLeft.red() < targetRed && colorLeft.blue() < targetBlue){ //drive back till we see blue line then stop
            leftFront.setPower(pow*dir);
            leftBack.setPower(pow*dir);
            rightBack.setPower(pow*dir);
            rightFront.setPower(pow*dir);

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
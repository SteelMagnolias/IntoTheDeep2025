package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "earlySeasonNetZone" , group = "Linear OpMode")
public class earlySeasonNetZone extends LinearOpMode {
    // declare motors and servos
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor armLeft;
    private DcMotor armRight;
    private DcMotor armEncoder;
    private CRServo intake;

    // sensors
    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;
    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    // variables
    double pow = 0.2;
    double armPow = 0.7;
    int targetRed = 2000;
    int targetBlue = 2000;
    int targetGreen = 2000;
    double buffer = 0.2;
    double width = 5.125;

    @Override
    public void runOpMode() throws InterruptedException {
        //map motors from configuration to motor names in code
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        intake = hardwareMap.get(CRServo.class, "intake");

        //Reverse Motor
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        armRight.setDirection(DcMotor.Direction.REVERSE);

        //encoder setup
        armEncoder = armRight;
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // sensors
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");

        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");

        //telemetry
        telemetry.addData("arm encoder", armEncoder.getCurrentPosition());

        telemetry.addData("distanceLeft:", distanceLeft.getDistance(DistanceUnit.INCH));
        telemetry.addData("distanceRight:", distanceRight.getDistance(DistanceUnit.INCH));

        telemetry.addData("colorLeft Red:", colorLeft.red());
        telemetry.addData("colorLeft Blue:", colorLeft.blue());
        telemetry.addData("colorLeft Green: ", colorLeft.green());

        telemetry.addData("Color Sensor colorRight Red:", colorRight.red());
        telemetry.addData("Color Sensor colorRight Blue:", colorRight.blue());
        telemetry.addData("Color Sensor colorRight Green: ", colorRight.green());

        telemetry.update();

        waitForStart();

        // turn counter-clockwise, line up with net line, looking for red/blue
        while(colorRight.red() < targetRed && colorRight.blue() < targetBlue){
            rightFront.setPower(pow);
            rightBack.setPower(pow);
            leftBack.setPower(pow/2);
        }
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);

        //may have to strafe

        //sample in basket
        arm(420);//change this
        intake(1,500);
        arm(0);

        strafeRight(3.4); // Strafes right to hit wall and turn
        strafeLeft(1.8); // Strafes left about 1 square to pass field samples
        turnCC(1.54); // Turns to face the general direction of the submersible

        // line up with submersible, strafe left, looking for white
        while(colorLeft.red() < targetRed || colorLeft.blue() < targetBlue || colorLeft.green() < targetGreen){
            leftFront.setPower(-pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(-pow);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        distanceDrive(6); // Moving forwards, stops before submersible

        arm(69); // Put arm up to park
    }


    private void driveForward (double t){
        leftFront.setPower(pow);
        leftBack.setPower(pow);
        rightFront.setPower(pow);
        rightBack.setPower(pow);
        sleep((long) t);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void driveBackwards (double t){
        leftFront.setPower(-pow);
        leftBack.setPower(-pow);
        rightFront.setPower(-pow);
        rightBack.setPower(-pow);
        sleep((long) t);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void strafeLeft (double t){
        leftFront.setPower(-pow);
        leftBack.setPower(pow);
        rightFront.setPower(pow);
        rightBack.setPower(-pow);
        sleep((long) t);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void strafeRight (double t){
        leftFront.setPower(pow);
        leftBack.setPower(-pow);
        rightFront.setPower(-pow);
        rightBack.setPower(pow);
        sleep((long) t);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void turnCC (double t){
        leftFront.setPower(-pow);
        leftBack.setPower(-pow);
        rightFront.setPower(pow);
        rightBack.setPower(pow);
        sleep((long) t);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void turnC (double t){
        leftFront.setPower(pow);
        leftBack.setPower(pow);
        rightFront.setPower(-pow);
        rightBack.setPower(-pow);
        sleep((long) t);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void drive (double lf, double lb, double rf, double rb, double t){
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
        sleep((long) t);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void distanceDrive (double dis){
        double DRValue = distanceRight.getDistance(DistanceUnit.INCH);
        double DLValue = distanceLeft.getDistance(DistanceUnit.INCH);
        double DAValue = (DRValue+DLValue)/2 - dis;

        while(DAValue > buffer || DAValue < -buffer){
            //variables
            DRValue = distanceRight.getDistance(DistanceUnit.INCH);
            DLValue = distanceLeft.getDistance(DistanceUnit.INCH);
            DAValue = (DRValue+DLValue)/2 - dis;

            // forwards backwards movement
            pow = 2*((DAValue + dis) / 30) - dis;

            //turning movement
            double dir = 1;
            if(DLValue > DRValue) dir = -1;
            double theta = dir * (90 - Math.abs (Math.atan (width / (DLValue - DRValue))));
            if (theta > 0) theta += 10;
            if (theta < 0) theta -= 10;
            double thetaPow = 2 * ((theta+100)/200) - 1;

            //set wheel power
            leftFront.setPower(pow - thetaPow);
            leftBack.setPower(pow - thetaPow);
            rightFront.setPower(pow + thetaPow);
            rightBack.setPower(pow + thetaPow);
        }

        //reset wheels and pow
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        pow = 4;
    }

    private void arm(int dis){
        while (armEncoder.getCurrentPosition() < dis - 25) { //move arm up
            armRight.setPower(armPow);
            armLeft.setPower(armPow);
        }
        while (armEncoder.getCurrentPosition() > dis + 25) { // move arm down
            armRight.setPower(-armPow);
            armLeft.setPower(-armPow);
        }
        armLeft.setPower(0);
        armLeft.setPower(0);
    }

    private void intake(int dir, double t){
        intake.setPower(pow*dir);
        sleep((long) t);
        intake.setPower(0);
    }
}
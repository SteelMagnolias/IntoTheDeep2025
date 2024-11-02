package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "earlySeasonObservationZone" , group = "Linear OpMode")
public class earlySeasonObservationZone extends LinearOpMode {

    // declare motors
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
    double pow = 0.4;
    double armPow = 0.7;
    double specimenDistance = 6; //inches

    @Override
    public void runOpMode() throws InterruptedException {
        //map motors from configuration to motor names in code
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
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
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armEncoder = armRight;

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

        telemetry.addData("test", 0);
        telemetry.update();

        leftFront.setPower(pow);


        distanceDrive(1, specimenDistance); // drive forwards to submersible

        // hang specimen
        arm(420);//change this
        intake(1,500);
        arm(0);

        distanceDrive(-1, 28);// drive till against wall 30 inches sub 2 for buffer
        strafeRight(4.7); //drive to other wall into observation zone to park
    }

    private void driveForward(double t) {
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

    private void driveBackwards(double t) {
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

    private void strafeLeft(double t) {
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

    private void strafeRight(double t) {
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

    private void turnCC(double t) {
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

    private void turnC(double t) {
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

    private void drive(double lf, double lb, double rf, double rb, double t) {
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

    private void distanceDrive(int dir, double dis) {
        double DRValue;
        double DLValue;

        if (dir > 0) { //forwards
            while (distanceRight.getDistance(DistanceUnit.INCH) < dis && distanceLeft.getDistance(DistanceUnit.INCH) < dis) { // both sensors within distance
                DRValue = distanceRight.getDistance(DistanceUnit.INCH);
                DLValue = distanceLeft.getDistance(DistanceUnit.INCH);
                if (DRValue >= DLValue + .5) { // if right if further forwards
                    leftFront.setPower(pow);
                    leftBack.setPower(pow);
                    rightFront.setPower(pow / 2);
                    rightBack.setPower(pow / 2);
                } else if (DLValue >= DRValue + .5) { // if left is further forwards
                    leftFront.setPower(pow / 2);
                    leftBack.setPower(pow / 2);
                    rightFront.setPower(pow);
                    rightBack.setPower(pow);
                } else { // if within 1 inch of eachother
                    leftFront.setPower(pow);
                    leftBack.setPower(pow);
                    rightFront.setPower(pow);
                    rightBack.setPower(pow);
                }
            }
        } else { // backwards
            while (distanceRight.getDistance(DistanceUnit.INCH) > dis && distanceLeft.getDistance(DistanceUnit.INCH) > dis) { // both sensors within distance
                DRValue = distanceRight.getDistance(DistanceUnit.INCH);
                DLValue = distanceLeft.getDistance(DistanceUnit.INCH);
                if (DRValue >= DLValue + .5) { // if right is further forwards
                    leftFront.setPower(-pow / 2);
                    leftBack.setPower(-pow / 2);
                    rightFront.setPower(-pow);
                    rightBack.setPower(-pow);
                } else if (DLValue >= DRValue + .5) { //if left is further forwards
                    leftFront.setPower(-pow);
                    leftBack.setPower(-pow);
                    rightFront.setPower(-pow / 2);
                    rightBack.setPower(-pow / 2);
                } else { // if within 1 inch of each other
                    leftFront.setPower(-pow);
                    leftBack.setPower(-pow);
                    rightFront.setPower(-pow);
                    rightBack.setPower(-pow);
                }
            }
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void arm(int dis) {
        while (armEncoder.getCurrentPosition() < dis - 25) { //move arm up
            armRight.setPower(armPow);
            armLeft.setPower(armPow);
            telemetry.addData("arm encoder", armEncoder.getCurrentPosition());
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


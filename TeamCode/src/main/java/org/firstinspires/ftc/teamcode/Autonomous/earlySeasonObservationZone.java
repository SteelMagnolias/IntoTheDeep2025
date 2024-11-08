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
    double armPow = 0.3;
    double specimenDistance = 16; //inches
    double armZone = 20;
    int targetBlue = 2000;
    int targetRed = 2000;
    int specimenArm = -1000;

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
        armEncoder = armLeft;

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

        intake.setPower(1); //hold specimen in

        waitForStart();

        targetBlue = colorRight.blue() + 1000;
        targetRed = colorRight.red() + 1000;

        turnC(250);

        distanceDrive(1, armZone); //drive forward away from wall

        arm(specimenArm); // put arm in hang position

        sleep(500); //wait for arm momentum to stop

        distanceDrive(1, specimenDistance); // drive to the submersible at hang distance

        arm(specimenArm - 1000); //hook specimen on bar

        sleep(500); //wait for arm momentum to stop

        intake(-1, 3000); // let go of specimen

        arm(specimenArm - 750); //bring arm up to not get stuck

        distanceDrive(-1, armZone);// drive till against wall 30 inches sub 2 for buffer

        turnC(1000); //turn so color sensors can see line

        strafeRight(1500); // drive to wall

        colorDrive(1); //drive into park zone
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
        double DAValue;

        while (distanceRight.getDistance(DistanceUnit.INCH) < dis - .2 || distanceLeft.getDistance(DistanceUnit.INCH) < dis - .2 || distanceRight.getDistance(DistanceUnit.INCH) > dis + .2 || distanceLeft.getDistance(DistanceUnit.INCH) > dis + .2) {
            DRValue = distanceRight.getDistance(DistanceUnit.INCH);
            DLValue = distanceLeft.getDistance(DistanceUnit.INCH);
            DAValue = (DRValue+DLValue)/2;

            if (DAValue > dis){ //if farther than want to be forwards
                pow = 0.2;
            } else if (DAValue < dis){ // if closer than want to be go backwards
                pow = -0.2;
            }

            if (DRValue > DLValue + .2 && pow > 0 || DLValue > DRValue + .2 && pow < 0) { // if right if further forwards
                leftFront.setPower(pow);
                leftBack.setPower(pow);
                rightFront.setPower(pow / 2);
                rightBack.setPower(pow / 2);
            } else if (DRValue > DLValue + .2 && pow < 0 || DLValue > DRValue + .2 && pow > 0) { // if left is further forwards
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

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        pow = 0.4;
    }

    private void arm(int dis) {
        while (armEncoder.getCurrentPosition() < dis - 30 || armEncoder.getCurrentPosition() > dis + 30) { //move arm up
            if(armEncoder.getCurrentPosition() < dis - 30) {
               armRight.setPower(armPow);
               armLeft.setPower(armPow);
            }
           else {
               armRight.setPower(-armPow);
               armLeft.setPower(-armPow);
           }
           telemetry.addData("arm encoder", armEncoder.getCurrentPosition());
           telemetry.update();
        }
        armLeft.setPower(0);
        armRight.setPower(0);
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

    private void intake(int dir, double t){
        intake.setPower(1*dir);
        sleep((long) t);
        intake.setPower(0);
    }
}


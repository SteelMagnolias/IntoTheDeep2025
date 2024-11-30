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
    double armPow = 0.6;
    double specimenDistance = 16.5; //inches
    double armZone = 20;
    int targetBlue = 2000;
    int targetRed = 2000;
    int specimenArm = -3615;
    double bufferW = 0.6;
    double bufferA = 30;

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

        intake.setPower(0.5); //hold specimen in

        //encoder setup
        armEncoder = armLeft;
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

        targetBlue = colorRight.blue() + 250;
        targetRed = colorRight.red() + 250;

        distanceDrive(armZone); //drive forward away from wall

        arm(specimenArm); // put arm in hang position

        sleep(500); //wait for arm momentum to stop

        distanceDrive(specimenDistance); // drive to the submersible at hang distance

        arm(specimenArm - 650); //hook specimen on bar

        sleep(500); //wait for arm momentum to stop

        intake(-1, 3000); // let go of specimen

        distanceDrive(armZone);// drive till against wall 30 inches sub 2 for buffer

        arm(-2500); //bring arm up to not get stuck

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

    private void distanceDrive(double dis) {
        double DRValue = distanceRight.getDistance(DistanceUnit.INCH);
        double DLValue = distanceLeft.getDistance(DistanceUnit.INCH);
        double DAValue = (DRValue+DLValue)/2 - dis;

        while(DAValue > bufferW || DAValue < - bufferW){
            //variables
            DRValue = distanceRight.getDistance(DistanceUnit.INCH);
            DLValue = distanceLeft.getDistance(DistanceUnit.INCH);
            DAValue = (DRValue+DLValue)/2 - dis;

            if (DRValue > DLValue + 5){
                leftFront.setPower(pow);
                leftBack.setPower(pow);
                rightFront.setPower(-pow);
                rightBack.setPower(-pow);
            } else  if (DRValue > DLValue + 5) {
                leftFront.setPower(-pow);
                leftBack.setPower(-pow);
                rightFront.setPower(pow);
                rightBack.setPower(pow);
            } else {
                double lp = (DLValue + 0.1 - dis) * 0.08;
                double rp = (DRValue + 0.1 - dis) * 0.08;

                leftFront.setPower(lp);
                leftBack.setPower(lp);
                rightFront.setPower(rp);
                rightBack.setPower(rp);
            }

            telemetry.addData("distanceLeft:", distanceLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("distanceRight:", distanceRight.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void arm(int dis) {
        while (armEncoder.getCurrentPosition() < dis - bufferA || armEncoder.getCurrentPosition() > dis + bufferA) {
            double armPos = armEncoder.getCurrentPosition();

            armPow = (armPos + 50 - dis) * -0.0025;

            armLeft.setPower(armPow);
            armRight.setPower(armPow);

           telemetry.addData("arm encoder", armEncoder.getCurrentPosition());
           telemetry.update();
        }
        armLeft.setPower(0);
        armRight.setPower(0);
    }

    public void colorDrive (int dir){
        pow = 0.2;
        while (colorRight.red() < targetRed && colorRight.blue() < targetBlue){ //drive back till we see blue line then stop
            leftFront.setPower(pow*dir);
            leftBack.setPower(pow*dir);
            rightBack.setPower(pow*dir);
            rightFront.setPower(pow*dir);

            telemetry.addData("colorRight Red:", colorRight.red());
            telemetry.addData("colorRight Blue:", colorRight.blue());

            telemetry.addData("target red", targetRed);
            telemetry.addData("target blue", targetBlue);

            telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        pow = 0.4;
    }

    private void intake(int dir, double t){
        intake.setPower(1*dir);
        sleep((long) t);
        intake.setPower(0);
    }
}


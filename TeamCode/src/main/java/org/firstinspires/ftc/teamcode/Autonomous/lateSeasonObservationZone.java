package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "lateSeasonObservationZone", group = "Iterative OpMode")
public class lateSeasonObservationZone extends OpMode {

    //declare motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor armLeft;
    private DcMotor armRight;
    private CRServo intake;

    //declare encoders
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;
    private DcMotor armEncoder;

    //declare sensors
    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;
    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    //stagnant variables
    double pow = 0.4;
    double armPow = 0.6;
    int step = 1;
    int targetBlue = 2000;
    int targetRed = 2000;
    int targetGreen = 2000;
    ElapsedTime intakeTimer = new ElapsedTime();

    //bot information
    double trackWidth = 20; //centimeters
    double trackWidthDelta = 0; //for tuning
    double yOffset = -13.5; //centimeters
    double yOffsetDelta = 0; //for tuning
    double leftWheelDiameter = 3.469; //centimeters
    double rightWheelDiameter = 3.315; //centimeters
    double backWheelDiameter = 3.471; //centimeters
    double leftWheelCircumference = Math.PI * leftWheelDiameter;
    double rightWheelCircumference = Math.PI * rightWheelDiameter;
    double backWheelCircumference = Math.PI * backWheelDiameter;
    double countsPerRotation = 8192;

    double[] pose = {0, 0, Math.toRadians(0)};

    double previousLeftEncoderPosition = 0;
    double previousRightEncoderPosition = 0;
    double previousBackEncoderPosition = 0;

    //other variables
    double bufferW = 0.6;
    double bufferA = 30;

    public void init () {
        //motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        intake = hardwareMap.get(CRServo.class, "intake");

        // sensors
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");

        //reverse motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        armRight.setDirection(DcMotor.Direction.REVERSE);

        //reset encoders
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set wheels to run seperate from the encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //connect encoders to the ports
        leftEncoder = leftFront;
        rightEncoder = rightFront;
        backEncoder = rightBack;
        armEncoder = armLeft;

        //set start postition - reverse depending the direction the encoder is facing
        previousLeftEncoderPosition = leftEncoder.getCurrentPosition();
        previousRightEncoderPosition = rightEncoder.getCurrentPosition();
        previousBackEncoderPosition = backEncoder.getCurrentPosition();

        intake.setPower(0.5);

        targetBlue = colorRight.blue() + 250;
        targetRed = colorRight.red() + 250;
        targetGreen = colorRight.green() + 250;
    }
    public void loop(){

            runOdometry();

            switch (step) {
                case 1:
                    //movement type
                    //if(pose[#] >= or <= # of centimeters or # of degrees){
                    //movement type (0)
                    step++;
                    //}
                    break;
                default:
                    drive(0, 0, 0, 0);
                    stop();
            }
    }

    private void runOdometry() {
        //odometry math
        //current encoder ticks
        double leftEncoderRawValue = leftEncoder.getCurrentPosition();
        double rightEncoderRawValue = rightEncoder.getCurrentPosition();
        double backEncoderRawValue = backEncoder.getCurrentPosition();

        //calculate the change from previous position to current encoder position and convert to centimeters
        double leftEncoderChange = ((leftEncoderRawValue - previousLeftEncoderPosition) / countsPerRotation) * leftWheelCircumference;
        double rightEncoderChange = ((rightEncoderRawValue - previousRightEncoderPosition) / countsPerRotation) * rightWheelCircumference;
        double backEncoderChange = ((backEncoderRawValue - previousBackEncoderPosition) / countsPerRotation) * backWheelCircumference;

        //find the change in robot angle by averageing both sides using subtraction due to opposite angles and then multiply by the radius to turn it into an angle
        double robotAngle = (leftEncoderChange - rightEncoderChange) / (trackWidth + trackWidthDelta);

        //find the change in x center by averaging the left and right encoder values
        double xCenter = (leftEncoderChange + rightEncoderChange) / 2;

        //find the change in x perpendicular by multiplying y offset by the robot angle and subtracting it from the back encoder
        double xPerpendicular = backEncoderChange - ((yOffset + yOffsetDelta) * robotAngle);

        //relate the change in x center to our position on the field using trig
        double xChange = xCenter * Math.cos(pose[2]) - xPerpendicular * Math.sin(pose[2]);

        //relate the change in x perpendicular to our position on the field using trig
        double yChange = xCenter * Math.sin(pose[2]) + xPerpendicular * Math.cos(pose[2]);

        //add our new location to the old one and set each pose equal to it.
        pose[0] += xChange;
        pose[1] += yChange;
        pose[2] += robotAngle;

        //set previous encoder position to current encoder position
        previousLeftEncoderPosition = leftEncoderRawValue;
        previousRightEncoderPosition = rightEncoderRawValue;
        previousBackEncoderPosition = backEncoderRawValue;
    }

    private void driveForward(double p){
        leftFront.setPower(p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(p);
    }

    private void driveBackwards(double p){
        leftFront.setPower(-p);
        leftBack.setPower(-p);
        rightFront.setPower(-p);
        rightBack.setPower(-p);
    }

    private void strafeLeft(double p){
        leftFront.setPower(-p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(-p);
    }

    private void strafeRight(double p){
        leftFront.setPower(p);
        leftBack.setPower(-p);
        rightFront.setPower(-p);
        rightBack.setPower(p);
    }

    private void drive (double fl, double bl, double fr, double br){
        leftFront.setPower(fl);
        leftBack.setPower(bl);
        rightFront.setPower(fr);
        rightBack.setPower(br);
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
        }
        armLeft.setPower(0);
        armRight.setPower(0);
    }

    public void colorDrive (int dir){
        pow = 0.2;
        while (colorLeft.red() < targetRed && colorLeft.blue() < targetBlue){ //drive back till we see blue line then stop
            leftFront.setPower(pow*dir);
            leftBack.setPower(pow*dir);
            rightBack.setPower(pow*dir);
            rightFront.setPower(pow*dir);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        pow = 0.4;
    }

    private void intake(int dir, double t){
        //reset timer in previous case
        intake.setPower(1*dir);
        if (intakeTimer.milliseconds() > t) {
            intake.setPower(0);
            step++;
        }
    }
}
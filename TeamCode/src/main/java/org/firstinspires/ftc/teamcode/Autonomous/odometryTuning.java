package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "odometryTuning", group = "Iterative OpMode")
public class odometryTuning extends OpMode {

    //declare motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    //declare encoders
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;


    //stagnant variables
    int step = 0;
    ElapsedTime odometryTimer = new ElapsedTime();
    ElapsedTime stepTimer = new ElapsedTime();
    double currentTime;
    double oP;
    double oI;
    double oD;
    double anglePow = 0;
    double desAngle = 0;
    double previousTime;
    double previousError;
    double angle;

    //bot information
    double trackWidth = 36.75; //centimeters
    double yOffset = 3.75; //centimeters
    double leftWheelDiameter = 4.732; //centimeters
    double rightWheelDiameter = 4.729; //centimeters
    double backWheelDiameter = 4.752; //centimeters
    double leftWheelCircumference = Math.PI * leftWheelDiameter;
    double rightWheelCircumference = Math.PI * rightWheelDiameter;
    double backWheelCircumference = Math.PI * backWheelDiameter;
    double countsPerRotation = 2000;

    double[] pose = {0, 0, Math.toRadians(0)};

    double previousLeftEncoderPosition = 0;
    double previousRightEncoderPosition = 0;
    double previousBackEncoderPosition = 0;

    //other variables
    double pow = 0.4;
    double trackWidthDelta = 0; //for tuning
    double yOffsetDelta = -1; //for tuning
    double Op = 0.07;
    double Oi = 0.0005;
    double Od = 0.5;

    public void init () {
        //motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //reverse motors
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //reset encoders
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        leftEncoder = leftBack;
        rightEncoder = rightFront;
        backEncoder = rightBack;

        //set start postition - reverse depending the direction the encoder is facing
        previousLeftEncoderPosition = leftEncoder.getCurrentPosition();
        previousRightEncoderPosition = rightEncoder.getCurrentPosition();
        previousBackEncoderPosition = backEncoder.getCurrentPosition();
    }

    public void loop(){
        boolean a2 = gamepad2.a;

        runOdometry();
        robotAnglePID();
        telemetry();

        if(a2 && stepTimer.milliseconds() > 1000){
            step++;
            stepTimer.reset();
        }

        switch (step) {
            case 1:
                driveForward(pow);
                if (pose[0] >= 40) {
                    drive(0,0,0,0);
                    step++;
                }
                break;
            case 3:
                driveBackwards(pow);
                if (pose[0] <= 0) {
                    drive(0,0,0,0);
                    step++;
                }
                break;
            case 5:
                strafeRight(pow);
                if (pose[1] >= 40) {
                    drive(0,0,0,0);
                    step++;
                }
                break;
            case 7:
                strafeLeft(pow);
                if (pose[1] <= 0) {
                    drive(0,0,0,0);
                    step++;
                }
                break;
            case 9:
                clockwise();
                desAngle = 360;
                if (pose[2] >= Math.toRadians(desAngle)) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 11:
                counterClockwise();
                desAngle = 90;
                if (pose[2] <= Math.toRadians(desAngle)) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 13:
                strafeRight(pow);
                if (pose[0] < -40){
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 15:
                strafeLeft(pow);
                if (pose[0] > 0) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 17:
                driveForward(pow);
                if(pose[1] > 40){
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 19:
                driveBackwards(pow);
                if (pose[1] < 0){
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 21:
                clockwise();
                desAngle = 360;
                if (pose[2] >= Math.toRadians(desAngle)) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 23:
                counterClockwise();
                desAngle = 180;
                if (pose[2] <= Math.toRadians(desAngle)) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 25:
                driveBackwards(pow);
                if (pose[0] >= 40) {
                    drive(0,0,0,0);
                    step++;
                }
                break;
            case 27:
                driveForward(pow);
                if (pose[0] <= 0) {
                    drive(0,0,0,0);
                    step++;
                }
                break;
            case 29:
                strafeLeft(pow);
                if (pose[1] >= 40) {
                    drive(0,0,0,0);
                    step++;
                }
                break;
            case 31:
                strafeRight(pow);
                if (pose[1] <= 0) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 33:
                clockwise();
                desAngle = 360;
                if (pose[2] >= Math.toRadians(desAngle)) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 35:
                counterClockwise();
                desAngle = 270;
                if (pose[2] <= Math.toRadians(desAngle)) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 37:
                strafeLeft(pow);
                if (pose[0] < -40){
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 39:
                strafeRight(pow);
                if (pose[0] > 0) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 41:
                driveBackwards(pow);
                if(pose[1] > 40){
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 43:
                driveForward(pow);
                if (pose[1] < 0){
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 45:
                clockwise();
                desAngle = 360;
                if (pose[2] >= Math.toRadians(desAngle)) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            case 47:
                counterClockwise();
                desAngle = 0;
                if (pose[2] <= Math.toRadians(desAngle)) {
                    drive(0,0,0,0);
                    step++;
                }
                break;

            default:
                drive(0, 0, 0, 0);
                stop();
        }
    }
    private void telemetry (){
        telemetry.addLine("Motor Powers");

        telemetry.addLine();

        telemetry.addData("Right Front Power: ", rightFront.getPower());
        telemetry.addData("Right Back Power: ", rightBack.getPower());
        telemetry.addData("Left Front Power: ", leftFront.getPower());
        telemetry.addData("Left Back Power: ", leftBack.getPower());

        telemetry.addLine();

        telemetry.addLine("Variables and Sensors");

        telemetry.addLine();

        telemetry.addData("Step", step);

        telemetry.addLine();

        telemetry.addData("Odometry X: ", pose [0]);
        telemetry.addData("Odometry Y: ", pose [1]);
        telemetry.addData("Odometry Rotation: ", Math.toDegrees(pose [2]));

        telemetry.update();
    }
    private void runOdometry() {
        //odometry math
        //current encoder ticks
        double leftEncoderRawValue = leftEncoder.getCurrentPosition();
        double rightEncoderRawValue = -rightEncoder.getCurrentPosition();
        double backEncoderRawValue = backEncoder.getCurrentPosition();

        telemetry.addData("leftEncoderRawValue", leftEncoderRawValue);
        telemetry.addData("rightEncoderRawValue", rightEncoderRawValue);
        telemetry.addData("backEncoderRawValue", backEncoderRawValue);

        //calculate the change from previous position to current encoder position and convert to centimeters
        double leftEncoderChange = ((leftEncoderRawValue - previousLeftEncoderPosition) / countsPerRotation) * leftWheelCircumference;
        double rightEncoderChange = ((rightEncoderRawValue - previousRightEncoderPosition) / countsPerRotation) * rightWheelCircumference;
        double backEncoderChange = ((backEncoderRawValue - previousBackEncoderPosition) / countsPerRotation) * backWheelCircumference;

        telemetry.addData("leftEncoderChange", leftEncoderChange);
        telemetry.addData("rightEncoderChange", rightEncoderChange);
        telemetry.addData("backEncoderChange", backEncoderChange);

        //find the change in robot angle by averageing both sides using subtraction due to opposite angles and then multiply by the radius to turn it into an angle
        double robotAngle = (leftEncoderChange - rightEncoderChange) / (trackWidth + trackWidthDelta);

        telemetry.addData("robotAngle", robotAngle);

        //find the change in x center by averaging the left and right encoder values
        double xCenter = (leftEncoderChange + rightEncoderChange) / 2;

        telemetry.addData("xCenter", xCenter);
        //find the change in x perpendicular by multiplying y offset by the robot angle and subtracting it from the back encoder
        double xPerpendicular = backEncoderChange - ((yOffset + yOffsetDelta) * robotAngle);

        telemetry.addData("xPerpendicular", xPerpendicular);

        //relate the change in x center to our position on the field using trig
        double xChange =  xCenter * Math.cos(pose[2]) - xPerpendicular * Math.sin(pose[2]);

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

        telemetry.addData("previousLeftEncoderPosition", previousLeftEncoderPosition);
        telemetry.addData("previousRightEncoderPosition", previousRightEncoderPosition);
        telemetry.addData("previousBackEncoderPosition", previousBackEncoderPosition);
    }

    private void robotAnglePID (){
        //PID on angle
        currentTime = odometryTimer.milliseconds();
        angle = desAngle - Math.toDegrees(pose [2]);
        oP = angle * Op;
        oI = Oi * (angle * (currentTime - previousTime));
        oD = Od * (angle - previousError) / (currentTime - previousTime);
        anglePow = (oP + oI +oD);
        if (anglePow > 0.7) anglePow = 0.7;
        if  (anglePow< -0.7) anglePow = -0.7;
        previousTime = currentTime;
        previousError = angle;
    }

    private void driveForward(double p){
        leftFront.setPower(p+anglePow);
        leftBack.setPower(p+anglePow);
        rightFront.setPower(p-anglePow);
        rightBack.setPower(p-anglePow);
    }

    private void driveBackwards(double p){
        leftFront.setPower(-p+anglePow);
        leftBack.setPower(-p+anglePow);
        rightFront.setPower(-p-anglePow);
        rightBack.setPower(-p-anglePow);
    }

    private void strafeLeft(double p){
        leftFront.setPower(-p+anglePow);
        leftBack.setPower(p+anglePow);
        rightFront.setPower(p-anglePow);
        rightBack.setPower(-p-anglePow);
    }

    private void strafeRight(double p){
        leftFront.setPower(p+anglePow);
        leftBack.setPower(-p+anglePow);
        rightFront.setPower(-p-anglePow);
        rightBack.setPower(p-anglePow);
    }

    private void clockwise (){
            leftFront.setPower(anglePow);
            leftBack.setPower(anglePow);
            rightFront.setPower(-anglePow);
            rightBack.setPower(-anglePow);
    }

    private void counterClockwise (){
        leftFront.setPower(anglePow);
        leftBack.setPower(anglePow);
        rightFront.setPower(-anglePow);
        rightBack.setPower(-anglePow);
    }

    private void drive (double dfl, double dbl, double dfr, double dbr){
        leftFront.setPower(dfl+anglePow);
        leftBack.setPower(dbl+anglePow);
        rightFront.setPower(dfr-anglePow);
        rightBack.setPower(dbr-anglePow);
    }
}
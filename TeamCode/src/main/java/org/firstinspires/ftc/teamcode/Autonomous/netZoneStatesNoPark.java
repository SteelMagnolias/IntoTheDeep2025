package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "netZoneStatesNoPark", group = "Iterative OpMode")
public class netZoneStatesNoPark extends OpMode {

    // motors & servos
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DcMotor armLeft;
    private DcMotor armRight;
    private DcMotor armSlide;

    private CRServo intake;

    // encoders
    private DcMotor armEncoder;
    private DcMotor slideEncoder;

    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;

    //sensors
    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;

    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    //general
    ElapsedTime PIDTimer = new ElapsedTime();
    ElapsedTime wTimer = new ElapsedTime();
    ElapsedTime aTimer = new ElapsedTime();

    double currentTime = 0;
    double previousTime = 0;

    int stepW = 1;
    int stepA = 1;
    int stepR = 1;

    double pow = 0.4;
    double armPow = 0.9;
    double slidePow = 0.9;
    double anglePow = 0;

    //color
    int targetRedLeft = 2000;
    int targetBlueLeft = 2000;

    int targetRedRight = 2000;
    int targetBlueRight = 2000;

    //distance
    double distanceErrorLeft = 0;
    double distanceErrorRight = 0;

    double previousDistanceErrorLeft = 0;
    double previousDistanceErrorRight = 0;

    double desDis;
    double disLeft;
    double disRight;
    double turnDis;

    //arm
    double armError = 0;
    double previousArmError = 0;
    double desArmPos;
    double armPos;

    double lengthError = 0;
    double previousLengthError = 0;
    double desLength;
    double armLength;

    //odometry
    double angleError;
    double desAngle = 0;
    double previousAngleError = 0;

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

    //tuning variables
    //arm PIDs
    double AP = 0.001;
    double AI = 0.000005;
    double AD = 0.005;

    double SP = 0.00015;
    double SI = 0;
    double SD = 0;

    //distance PID
    double DP = 0.04;
    double DI = 0.00000005;
    double DD = 0.05;

    //odometry rotation PID
    double OP = 0.06;
    double OI = 0;
    double OD = 0.5;

    //other variables
    double bufferD = 0.6;
    double bufferA = 30;
    double bufferL = 3;
    double bufferO = 1;
    double bufferOT = 3;

    double trackWidthDelta = 0;
    double yOffsetDelta = -1;

    public void init() {
        //motors and servos
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armSlide = hardwareMap.get(DcMotor.class, "armSlide");

        intake = hardwareMap.get(CRServo.class, "intake");

        //reversals
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        armSlide.setDirection(DcMotor.Direction.REVERSE);
        armLeft.setDirection(DcMotor.Direction.REVERSE);
        armRight.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.REVERSE);

        //motor set up
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoder set up
        leftEncoder = leftBack;
        rightEncoder = rightFront;
        backEncoder = rightBack;

        armEncoder = armLeft;
        slideEncoder = armSlide;

        previousLeftEncoderPosition = leftEncoder.getCurrentPosition();
        previousRightEncoderPosition = -rightEncoder.getCurrentPosition();
        previousBackEncoderPosition = backEncoder.getCurrentPosition();

        //sensors
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");

        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");

        //set target
        targetBlueRight = colorRight.blue() + 400;
        targetRedRight = colorRight.red() + 400;

        targetBlueLeft = colorLeft.blue() + 400;
        targetRedLeft = colorLeft.red() + 400;

        intake.setPower(0.15);

        telemetry.addLine("ready");
    }

    @Override
    public void loop() {
        currentTime = PIDTimer.milliseconds();

        runOdometry();
        robotAnglePID();

        switch (stepW) {
            case 1: // strafe away from wall
                strafeRight(pow);
                if(pose[1] > 17){
                    drive(0,0,0,0);
                    stepW++;
                }
                break;

            case 2: // drive forward
                driveForward(0.3);
                if(pose[0] > 30){
                    drive(0,0,0,0);
                    stepW++;
                    desAngle = -45;
                }
                break;

            case 3: // turn to 45 deg
                turn();
                stepA(bufferOT, angleError);
                stepW(bufferOT, angleError);
                break;

            case 4: // wait for arm
                drive(0,0,0,0);
                break;

            case 5: // turn
                desAngle = 0;
                stepW++;
                break;

            case 6: // strafe to line up with block
                turn();
                stepW(bufferOT, angleError);
                break;

            case 7: // arm stuff
                driveBackwards(pow);
                if(pose[0] < 0){
                    drive (0,0,0,0);
                    stepW++;
                }
                break;

            case 8: // strafe back to priot position
                strafeLeft(pow);
                if(pose [1] < 3){
                    drive(0,0,0,0);
                    stepW++;
                }
                break;

            default:

                drive(0, 0, 0, 0);

                break;
        }
        if(stepA < 3 || stepA > 4) {
            arm();
        }

        if (stepA < 5) {
            slide();
        }

        switch (stepA) {
            case 1: //lift arm
                desArmPos = 2250;
                stepA(bufferA, armError);
                break;

            case 2: // put wrist in place
                desLength = -5000;
                break;

            case 3: //drop block
                armLeft.setPower(-0.5);
                armRight.setPower(-0.5);
                if(armEncoder.getCurrentPosition() < -3500){
                    stepA++;
                    armLeft.setPower(0);
                    armRight.setPower(0);
                }
                break;

            case 4: //lower arm on block
                intake.setPower(-1);
                if(wTimer.milliseconds() > 2500){
                    intake.setPower(0);
                    stepA++;
                    aTimer.reset();
                }
                break;

            case 5: // grab block
                armSlide.setPower(-0.3);
                if(slideEncoder.getCurrentPosition() > -150 || aTimer.milliseconds() > 2500){
                    stepA++;
                    armSlide.setPower(-0.15);
                }
                break;

            case 6: //lift arm
                desArmPos = 0;
                stepW(bufferA, armError);
                stepA(bufferA, armError);
                break;

            default:

                arm(0, 0);

                break;
        }

        telemetry();
        previousTime = currentTime;
    }

    private void telemetry (){
        telemetry.addLine("Motor Powers");

        telemetry.addLine();

        telemetry.addData("Right Front Power: ", rightFront.getPower());
        telemetry.addData("Right Back Power: ", rightBack.getPower());
        telemetry.addData("Left Front Power: ", leftFront.getPower());
        telemetry.addData("Left Back Power: ", leftBack.getPower());

        telemetry.addLine();

        telemetry.addData("Arm Right Power: ", armRight.getPower());
        telemetry.addData("Arm Left Power: ", armLeft.getPower());
        telemetry.addData("Arm Slide Power: ", armSlide.getPower());
        telemetry.addData("Intake Power: ", intake.getPower());

        telemetry.addLine();

        telemetry.addLine("Variables and Sensors");

        telemetry.addLine();

        telemetry.addData("Step Wheels", stepW);
        telemetry.addData("Step Arm", stepA);

        telemetry.addLine();

        telemetry.addData("Odometry X: ", pose [0]);
        telemetry.addData("Odometry Y: ", pose [1]);
        telemetry.addData("Odometry Rotation: ", Math.toDegrees(pose [2]));

        telemetry.addLine();

        telemetry.addData("Distance Right", distanceRight.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Left", distanceLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("turn Distance", turnDis);

        telemetry.addLine();

        telemetry.addData("Color Left Red:", colorLeft.red());
        telemetry.addData("Color Left Blue:", colorLeft.blue());
        telemetry.addData("Color Right Red:", colorRight.red());
        telemetry.addData("Color Right Blue:", colorRight.blue());

        telemetry.addLine();

        telemetry.addData("target left red", targetRedLeft);
        telemetry.addData("target left Blue", targetBlueLeft);
        telemetry.addData("target right red", targetRedRight);
        telemetry.addData("target right blue", targetBlueRight);

        telemetry.addLine();

        telemetry.addData("Arm Encoder", -armEncoder.getCurrentPosition());
        telemetry.addData("Arm Length", slideEncoder.getCurrentPosition());

        telemetry.update();
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

    private void turn (){
        leftFront.setPower(anglePow);
        leftBack.setPower(anglePow);
        rightFront.setPower(-anglePow);
        rightBack.setPower(-anglePow);
    }

    private void drive (double dfl, double dbl, double dfr, double dbr){
        leftFront.setPower(dfl+anglePow);
        rightFront.setPower(dfr-anglePow);
        rightBack.setPower(dbr-anglePow);
        leftBack.setPower(dbl+anglePow);
    }

    private void arm(double ap, double alp) {
        armLeft.setPower(ap);
        armRight.setPower(ap);

        armSlide.setPower(alp);
    }

    private void runOdometry() {
        //odometry math
        //current encoder ticks
        double leftEncoderRawValue = leftEncoder.getCurrentPosition();
        double rightEncoderRawValue = -rightEncoder.getCurrentPosition();
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
    }

    private void robotAnglePID (){
        //PID on angle
        angleError = desAngle - Math.toDegrees(pose [2]);
        anglePow = ((angleError * OP) + (OI * (angleError * (currentTime - previousTime))) + (OD * (angleError - previousAngleError) / (currentTime - previousTime)));
        if (anglePow > 0.7) anglePow = 0.7;
        if  (anglePow < -0.7) anglePow = -0.7;
        previousAngleError = angleError;
    }

    private void distanceDrive (){
        disLeft = distanceLeft.getDistance(DistanceUnit.CM);
        distanceErrorLeft = disLeft - desDis;

        disRight = distanceRight.getDistance(DistanceUnit.CM);
        distanceErrorRight = disRight - desDis;

        double powLeft = ((distanceErrorLeft * DP) + (DI * (distanceErrorLeft * (currentTime - previousTime))) + (DD * (distanceErrorLeft - previousDistanceErrorLeft) / (currentTime - previousTime)));
        double powRight = ((distanceErrorRight * DP) + (DI * (distanceErrorRight * (currentTime - previousTime))) + (DD * (distanceErrorRight - previousDistanceErrorRight) / (currentTime - previousTime)));

        rightBack.setPower(powRight);
        rightFront.setPower(powRight);
        leftBack.setPower(powLeft);
        leftFront.setPower(powLeft);

        previousDistanceErrorLeft = distanceErrorLeft;
        previousDistanceErrorRight = distanceErrorRight;
    }

    private void arm () {
        armPos = -armEncoder.getCurrentPosition();
        armError = armPos - desArmPos;

        armPow = ((armError * AP) + (AI * (armError * (currentTime - previousTime))) + (AD * (armError - previousArmError) / (currentTime - previousTime)));

        armLeft.setPower(armPow);
        armRight.setPower(armPow);

        previousArmError = armError;
    }

    private void slide () {
        armLength = slideEncoder.getCurrentPosition(); // add math for ticks to cm
        lengthError = armLength - desLength;

        slidePow = ((lengthError * SP) + (SI * (lengthError * (currentTime - previousTime))) + (SD * (lengthError - previousLengthError) / (currentTime - previousTime)));

        if(slidePow < -0.4) slidePow = -0.4;
        if(slidePow > 0.6) slidePow = 0.6;

        armSlide.setPower(slidePow);

        previousLengthError = lengthError;
    }

    private void stepW(double buffer, double errorValue){
        if (Math.abs(errorValue) < buffer && wTimer.milliseconds() > 50){
            stepW++;
        } else {
            wTimer.reset();
        }
    }

    private void stepA(double buffer, double errorValue){
        if (Math.abs(errorValue) < buffer && aTimer.milliseconds() > 50){
            stepA++;
        } else {
            aTimer.reset();
        }
    }
}
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "odometryTuning", group = "Iterative OpMode")
public class autonTest extends OpMode {

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
    private DistanceSensor distanceSide;
    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;

    private ColorSensor colorLeft;
    private ColorSensor colorRight;

    //general
    ElapsedTime PIDTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();

    double currentTime;
    double previousTime;

    int stepW = 1;
    int stepA = 1;

    double pow = 0.5;
    double armPow = 0.9;
    double slidePow = 0.9;

    //color
    int targetRedLeft = 2000;
    int targetBlueLeft = 2000;

    int targetRedRight = 2000;
    int targetBlueRight = 2000;

    //distance
    double distanceErrorLeft = 0;
    double distanceErrorRight = 0;
    double distanceErrorSide = 0;
    double previousDistanceErrorLeft = 0;
    double previousDistanceErrorRight = 0;
    double previousDistanceErrorSide = 0;
    double desDis;
    double disLeft;
    double disRight;
    double disSide;

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
    double x;
    double y;
    double angle;

    double previousAngle = 0;

    double trackWidth = 20; //centimeters
    double yOffset = -13.5; //centimeters
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

    //tuning variables
    //arm PIDs
    double AP = 0.00535;
    double AI = 0.000002;
    double AD = 0.5;

    double SP = 1;
    double SI = 0;
    double SD = 0;

    //distance PIDs
    double DP = 1;
    double DI = 0;
    double DD = 0;

    double DSP = 1;
    double DSI = 0;
    double DSD = 0;

    //odometry rotation PID
    double OP = 1;
    double OI = 0;
    double OD = 0;

    //other variables
    double bufferD = 0.6;
    double bufferA = 30;
    double bufferO = 1;
    double bufferOT = 3;

    double trackWidthDelta = 0;
    double yOffsetDelta = 0;

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

        //motor set up
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
        leftEncoder = leftFront;
        rightEncoder = rightFront;
        backEncoder = rightBack;
        armEncoder = armLeft;
        slideEncoder = armSlide;

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sensors
        distanceSide = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");

        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");

        //set target
        targetBlueRight = colorRight.blue() + 250;
        targetRedRight = colorRight.red() + 250;

        targetBlueLeft = colorLeft.blue() + 250;
        targetRedLeft = colorLeft.red() + 250;
    }

    @Override
    public void loop() {

        currentTime = PIDTimer.milliseconds();

        switch (stepW) {
            case 1:


                break;

            default:

                drive(0, 0, 0, 0);

                break;
        }

        switch (stepA) {
            case 1:

                break;

            default:

                arm(0, 0);

                break;
        }

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
        telemetry.addData("Distance Side", distanceSide.getDistance(DistanceUnit.CM));

        telemetry.addLine();

        telemetry.addData("Color Left Red:", colorLeft.red());
        telemetry.addData("Color Left Blue:", colorLeft.blue());
        telemetry.addData("Color Right Red:", colorRight.red());
        telemetry.addData("Color Right Blue:", colorRight.blue());

        telemetry.addLine();

        telemetry.addData("Arm Encoder", armEncoder.getCurrentPosition());
        telemetry.addData("Arm Length", slideEncoder.getCurrentPosition());

        telemetry.update();
    }
    private void drive(double dfl, double dbl, double dfr, double dbr) {
        leftFront.setPower(dfl);
        leftBack.setPower(dbl);
        rightFront.setPower(dfr);
        rightBack.setPower(dbr);
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

    private void odometryDrive(double desX, double desY, double desRobotAngle) {
        pow = 0.5;

        x = desX - pose[0];
        y = desY - pose[1];
        angle = desRobotAngle - Math.toDegrees(pose[2]);


        double c = Math.hypot(x, y); // find length of hypot using tan of triangle made by x and y
        double perct = pow * c; // scale by max power
        double theta;

        // determine quandrant
        if (x <= 0 && y >= 0) {
            theta = Math.atan(Math.abs(x) / Math.abs(y));
            theta += (Math.PI / 2);
        } else if (x < 0 && y <= 0) {
            theta = Math.atan(Math.abs(y) / Math.abs(x));
            theta += (Math.PI);
        } else if (x >= 0 && y < 0) {
            theta = Math.atan(Math.abs(x) / Math.abs(y));
            theta += (3 * Math.PI / 2);
        } else {
            theta = Math.atan(Math.abs(y) / Math.abs(x));
        }


        double dir = 1; // default of direction being forward
        if (theta >= Math.PI) { // if we have an angle other 180 degrees on unit circle, then direction is backward
            theta -= Math.PI;
            dir = -1;
        }

        // calculate power of front right wheel
        double fr = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4)); // wheels move on a 45 degree angle, find the ratio of where we want to drive to where we need to be
        if (fr > 1) fr = 1; // cap speeds at 1 and -1
        if (fr < -1) fr = -1;
        fr = (perct * fr); // scale by power

        // calculate power of back left wheel, wheels move on 45 degree angles, find the ratio between where we are and where we should be
        double bl = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4));
        if (bl > 1) bl = 1; // cap speeds at 1 and -1
        if (bl < -1) bl = -1;
        bl = (perct * bl); // scale by power

        // calculate power of front left wheel, wheels move on 45 degree angles, find the ratio between where we are and where we should be
        double fl = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (fl > 1) fl = 1; // cap powers at 1 and -1
        if (fl < -1) fl = -1;
        fl = (perct * fl); // scale by power

        // calculate power of back right wheel, wheels move on 45 degree angles, find the ratio between where we are and where we should be
        double br = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (br > 1) br = 1; // cap powers at 1 and -1
        if (br < -1) br = -1;
        br = (perct * br); // scale by power

        //PID on angle
        double anglePow = ((angle * OP) + (OI * (angle * (currentTime - previousTime))) + (OD * (angle - previousAngle) / (currentTime - previousTime)));

        // set power of wheels and apply any rotation
        leftFront.setPower(fl + anglePow);
        leftBack.setPower(bl + anglePow);
        rightFront.setPower(fr - anglePow);
        rightBack.setPower(br - anglePow);

        previousAngle = angle;
    }

    private void distanceDrive (){
        disLeft = distanceLeft.getDistance(DistanceUnit.CM);
        distanceErrorLeft = disLeft - desDis;

        disRight =distanceRight.getDistance(DistanceUnit.CM);
        distanceErrorRight = disRight - desDis;

        double powLeft = ((distanceErrorLeft * DP) + (DI * (distanceErrorLeft * (currentTime - previousTime))) + (DD * (distanceErrorLeft - previousDistanceErrorLeft) / (currentTime - previousTime)));
        double powRight = ((distanceErrorRight * DP) + (DI * (distanceErrorRight * (currentTime - previousTime))) + (DD * (distanceErrorLeft - previousDistanceErrorRight) / (currentTime - previousTime)));

         rightBack.setPower(powRight);
         rightFront.setPower(powRight);
         leftBack.setPower(powLeft);
         leftFront.setPower(powLeft);

         previousDistanceErrorLeft = distanceErrorLeft;
         previousDistanceErrorRight = distanceErrorRight;
    }

    private void distanceSide (){
        disSide = distanceSide.getDistance(DistanceUnit.CM);
        distanceErrorSide = disSide - desDis;

        pow = ((distanceErrorSide * DSP) + (DSI * (distanceErrorSide * (currentTime - previousTime))) + (DSD * (distanceErrorSide - previousDistanceErrorSide) / (currentTime - previousTime)));

        rightBack.setPower(pow);
        rightFront.setPower(pow);
        leftBack.setPower(pow);
        leftFront.setPower(pow);

        previousDistanceErrorSide = distanceErrorSide;
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

        armLeft.setPower(slidePow);
        armRight.setPower(slidePow);

        previousLengthError = lengthError;
    }
}
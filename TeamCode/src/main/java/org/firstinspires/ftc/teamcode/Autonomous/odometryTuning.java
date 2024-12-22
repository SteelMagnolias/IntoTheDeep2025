package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "odonetryTuning", group = "Iterative OpMode")
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
    double pow = 0.4;
    int step = 1;
    ElapsedTime odometryTimer = new ElapsedTime();
    double currentTime;
    double oP;
    double oI;
    double oD;
    double previousTime;
    double previousError;

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
    double bufferO = 2.5;
    double bufferOT = 5;
    double oKp = 1;
    double oKi = 1;
    double oKd = 1;

    public void init () {
        //motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //reverse motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        //reset encoders
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        //set start postition - reverse depending the direction the encoder is facing
        previousLeftEncoderPosition = leftEncoder.getCurrentPosition();
        previousRightEncoderPosition = rightEncoder.getCurrentPosition();
        previousBackEncoderPosition = backEncoder.getCurrentPosition();
    }
    public void loop(){

        runOdometry();

        switch (step) {
            case 1:

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

    private void odometryDrive (double desX, double desY, double desRobotAngle){
        pow = 0.5;

        double x = pose [0] - desX;
        double y = pose [1] - desY;
        double angle = Math.toDegrees(pose[2]) - desRobotAngle;


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


        telemetry.addData("pow", pow);
        telemetry.addData("dir", dir);
        telemetry.addData("c", c);
        telemetry.addData("theta", theta);


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

        // add power for each wheel
        telemetry.addData("fl", fl);
        telemetry.addData("fr", fr);
        telemetry.addData("bl", bl);
        telemetry.addData("br", br);


        telemetry.addData("rlf", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rrf", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbl", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbr", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));

        //PID on angle
        currentTime = odometryTimer.milliseconds();
        oP = angle * oKp;
        oI = oKi * (angle * (currentTime - previousTime));
        oD = oKd * (angle - previousError) / (currentTime - previousTime);
        double anglePow = (oP + oI +oD);
        previousTime = currentTime;
        previousError = angle;

        // set power of wheels and apply any rotation
        leftFront.setPower(fl + anglePow);
        leftBack.setPower(bl + anglePow);
        rightFront.setPower(fr - anglePow);
        rightBack.setPower(br - anglePow);
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

    private void drive (double dfl, double dbl, double dfr, double dbr){
        leftFront.setPower(dfl);
        leftBack.setPower(dbl);
        rightFront.setPower(dfr);
        rightBack.setPower(dbr);
    }
}
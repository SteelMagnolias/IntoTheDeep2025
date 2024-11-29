package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "lateSeasonObservationZone", group = "Iterative OpMode")
public class lateSeasonObservationZone extends LinearOpMode {

    //declare motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    //declare encoders
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;

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

    int step = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //reverse motors

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

        waitForStart();

        while (!isStopRequested()) {

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
                    drive(0, 0,0,0);
                    stop();
            }
        }
    }

    private void runOdometry() {
        //odometry math

        double leftEncoderRawValue = leftEncoder.getCurrentPosition();
        double rightEncoderRawValue = rightEncoder.getCurrentPosition();
        double backEncoderRawValue = backEncoder.getCurrentPosition();
        //current encoder ticks

        double leftEncoderChange = ((leftEncoderRawValue - previousLeftEncoderPosition) / countsPerRotation) * leftWheelCircumference;
        double rightEncoderChange = ((rightEncoderRawValue - previousRightEncoderPosition) / countsPerRotation) * rightWheelCircumference;
        double backEncoderChange = ((backEncoderRawValue - previousBackEncoderPosition) / countsPerRotation) * backWheelCircumference;
        //calculate the change from previous position to current encoder position and convert to centimeters

        double robotAngle = (leftEncoderChange - rightEncoderChange) / (trackWidth + trackWidthDelta);
        //find the change in robot angle by averageing both sides using subtraction due to opposite angles and then multiply by the radius to turn it into an angle

        double xCenter = (leftEncoderChange + rightEncoderChange) / 2;
        //find the change in x center by averaging the left and right encoder values

        double xPerpendicular = backEncoderChange - ((yOffset + yOffsetDelta) * robotAngle);
        //find the change in x perpendicular by multiplying y offset by the robot angle and subtracting it from the back encoder

        double xChange = xCenter * Math.cos(pose[2]) - xPerpendicular * Math.sin(pose[2]);
        //relate the change in x center to our position on the field using trig

        double yChange = xCenter * Math.sin(pose[2]) * xPerpendicular * Math.cos(pose[2]);
        //relate the change in x perpendicular to our position on the field using trig

        pose[0] += xChange;
        pose[1] += yChange;
        pose[2] += robotAngle;
        //add our new location to the old one and set each pose equal to it.
    }

    private void forward(double p){
        leftFront.setPower(p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(p);
    }

    private void backwards(double p){
        leftFront.setPower(-p);
        leftBack.setPower(-p);
        rightFront.setPower(-p);
        rightBack.setPower(-p);
    }

    private void left(double p){
        leftFront.setPower(-p);
        leftBack.setPower(p);
        rightFront.setPower(p);
        rightBack.setPower(-p);
    }

    private void right(double p){
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
}
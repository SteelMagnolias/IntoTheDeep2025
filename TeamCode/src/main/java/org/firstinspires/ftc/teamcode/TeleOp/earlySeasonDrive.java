package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "earlySeasonDrive" , group = "Iterative Opmode")
public class earlySeasonDrive extends OpMode {
    // declare motors and servos
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor armLeft;
    private DcMotor armRight;
    private DcMotor armEncoder;
    private CRServo intake;

    // variables
    double pow;
    double armPow;
    double theta; //angle of wheels joystick
    double desArmPos;
    double armPos;
    int shake = 0;
    int shake2 = 0;


    public void init() {
        //map motors from configuration to motor names in code
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        intake = hardwareMap.get(CRServo.class, "intake");

        //reverse motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        armRight.setDirection(DcMotor.Direction.REVERSE);

        //encoder setup
        armEncoder = armLeft;
    }

    public void loop () {
        //main code

        //gamepad variables get data from gamepads
        // gamepad 1
        double lefty1 = -(gamepad1.left_stick_y); // this is the value of gamepad1's left joystick y value
        double leftx1 = gamepad1.left_stick_x; // this is the value of gamepad1's left joystick x value
        double rightx1 = gamepad1.right_stick_x; // this is the value of gamepad1's right joystick x value
        double righty1 = -(gamepad1.right_stick_y); // this the value of gamepad1's right joystick y value
        boolean buttonUp1 = gamepad1.dpad_up; // this is the value of gamepad1's up button on the dpad
        boolean buttonDown1 = gamepad1.dpad_down; // this is the value of gamepad1's down button on the dpad
        boolean buttonLeft1 = gamepad1.dpad_left; // this is the value of the gamepad1's left button on the dpad
        boolean buttonRight1 = gamepad1.dpad_right; // this is the value of the gamepad1's right button on the dpad
        boolean lb1 = gamepad1.left_bumper; // this is the value of the gamepad1's left bumper
        boolean rb1 = gamepad1.right_bumper; // this is the value of the gamepad1's right bumper
        boolean a1 = gamepad1.a; // this is the value of the a button on gamepad1
        boolean x1 = gamepad1.x; // this is the value of the x button on gamepad1
        boolean y1 = gamepad1.y; // this is the value of the y button on gamepad1
        boolean b1 = gamepad1.b; // this is the value of the b button on gamepad1

        // gamepad 2
        double lefty2 = -(gamepad2.left_stick_y); // this is the value of gamepad2's left joystick y value
        double leftx2 = gamepad2.left_stick_x; // this is the value of gamepad2's left joystick x value
        double rightx2 = gamepad2.right_stick_x; // this is the value of gamepad2's right joystick x value
        double righty2 = -(gamepad2.right_stick_y); // this the value of gamepad2's right joystick y value
        boolean buttonUp2 = gamepad2.dpad_up; // this is the value of gamepad2's up button on the dpad
        boolean buttonDown2 = gamepad2.dpad_down; // this is the value of gamepad2's down button on the dpad
        boolean buttonLeft2 = gamepad2.dpad_left; // this is the value of the gamepad2's left button on the dpad
        boolean buttonRight2 = gamepad2.dpad_right; // this is the value of the gamepad2's right button on the dpad
        boolean lb2 = gamepad2.left_bumper; // this is the value of the gamepad2's left bumper
        boolean rb2 = gamepad2.right_bumper; // this is the value of the gamepad2's right bumper
        boolean a2 = gamepad2.a; // this is the value of the a button on gamepad2
        boolean x2 = gamepad2.x; // this is the value of the x button on gamepad2
        boolean y2 = gamepad2.y; // this is the value of the y button on gamepad2
        boolean b2 = gamepad2.b; // this is the value of the b button on gamepad2
        boolean back2 = gamepad2.back;

        // gamepad telemetry
        // gamepad 1
        telemetry.addData("Gamepad:", 1);
        telemetry.addData("lefty1", lefty1);
        telemetry.addData("leftx1", leftx1);
        telemetry.addData("righty1", righty1);
        telemetry.addData("rightx1", rightx1);
        telemetry.addData("buttonUp1", buttonUp1);
        telemetry.addData("buttonDown1", buttonDown1);
        telemetry.addData("buttonLeft1", buttonLeft1);
        telemetry.addData("buttonRight1", buttonRight1);
        telemetry.addData("lb1", lb1);
        telemetry.addData("rb1", rb1);
        telemetry.addData("a1", a1);
        telemetry.addData("b1", b1);
        telemetry.addData("x1", x1);
        telemetry.addData("y1", y1);

        // gamepad 2
        telemetry.addData("Gamepad:", 2);
        telemetry.addData("lefty2", lefty2);
        telemetry.addData("leftx2", leftx2);
        telemetry.addData("righty2", righty2);
        telemetry.addData("rightx2", rightx2);
        telemetry.addData("buttonUp2", buttonUp2);
        telemetry.addData("buttonDown2", buttonDown2);
        telemetry.addData("buttonLeft2", buttonLeft2);
        telemetry.addData("buttonRight2", buttonRight2);
        telemetry.addData("lb2", lb2);
        telemetry.addData("rb2", rb2);
        telemetry.addData("a2", a2);
        telemetry.addData("b2", b2);
        telemetry.addData("x2", x2);
        telemetry.addData("y2", y2);

        //wheels code
        if (a1) pow = 1; // turbo mode
        else pow =0.9; // regular speed
        double c = Math.hypot(leftx1, lefty1); // find length of hypot using tan of triangle made by x and y
        double perct = pow * c; // scale by max power
        if (c <= .1) {
            perct = 0; // if we are less than .1 power, than just don't move since we are in dead zone
            shake = 0;
        }


        // determine quandrant
        if (leftx1 <= 0 && lefty1 >= 0) {
            theta = Math.atan(Math.abs(leftx1) / Math.abs(lefty1));
            theta += (Math.PI / 2);
        } else if (leftx1 < 0 && lefty1 <= 0) {
            theta = Math.atan(Math.abs(lefty1) / Math.abs(leftx1));
            theta += (Math.PI);
        } else if (leftx1 >= 0 && lefty1 < 0) {
            theta = Math.atan(Math.abs(leftx1) / Math.abs(lefty1));
            theta += (3 * Math.PI / 2);
        } else {
            theta = Math.atan(Math.abs(lefty1) / Math.abs(leftx1));
        }


        double dir = 1; // default of direction being forward
        if (theta >= Math.PI) { // if we have an angle other 180 degrees on unit circle, then direction is backward
            theta -= Math.PI;
            dir = -1;
        }
        //if (leftx1 <= 0 && lefty1 >= 0 || leftx1 >= 0 && lefty1 <= 0){
        //   theta += (Math.PI/2);
        //}


        telemetry.addData("pow", pow);
        telemetry.addData("dir", dir);
        telemetry.addData("c", c);
        telemetry.addData("theta", theta);


        // calculate power of front right wheel
        double fr = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4)); // wheels move on a 45 degree angle, find the ratio of where we want to drive to where we need to be
        if (fr > 1) fr = 1; // cap speeds at 1 and -1
        if (fr < -1) fr = -1;
        fr = (perct * fr); // scale by power
        if (leftx1 == 0 && lefty1 == 0) fr = 0; // if no joystick movement stop


        // calculate power of back left wheel, wheels move on 45 degree angles, find the ratio between where we are and where we should be
        double bl = dir * ((theta - (Math.PI / 4)) / (Math.PI / 4));
        if (bl > 1) bl = 1; // cap speeds at 1 and -1
        if (bl < -1) bl = -1;
        bl = (perct * bl); // scale by power
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) bl = 0; // if no joystick movement, stop wheel


        // calculate power of front left wheel, wheels move on 45 degree angles, find the ratio between where we are and where we should be
        double fl = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (fl > 1) fl = 1; // cap powers at 1 and -1
        if (fl < -1) fl = -1;
        fl = (perct * fl); // scale by power
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) fl = 0; // if no joystick movement, stop wheel


        // calculate power of back right wheel, wheels move on 45 degree angles, find the ratio between where we are and where we should be
        double br = -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4));
        if (br > 1) br = 1; // cap powers at 1 and -1
        if (br < -1) br = -1;
        br = (perct * br); // scale by power
        if (leftx1 < .1 && leftx1 > -.1 && lefty1 < .1 && lefty1 > -.1) br = 0; // if no joystick movement, stop


        // add power for each wheel
        telemetry.addData("fl", fl);
        telemetry.addData("fr", fr);
        telemetry.addData("bl", bl);
        telemetry.addData("br", br);


        telemetry.addData("rlf", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rrf", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbl", dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));
        telemetry.addData("rbr", -dir * ((theta - (3 * Math.PI / 4)) / (Math.PI / 4)));




        // set power of wheels and apply any rotation
        leftFront.setPower(fl + rightx1);
        leftBack.setPower(bl + rightx1);
        rightFront.setPower(fr - rightx1);
        rightBack.setPower(br - rightx1);


        // Below: precision (slower) movement
        pow = 0.3;
        if (buttonUp1) {
            // slowly moves forwards
            leftFront.setPower(pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(pow);
        } else if (buttonDown1) {
            // slowly moves backwards
            leftFront.setPower(-pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(-pow);
        } else if (buttonRight1) {
            // slowly moves right
            leftFront.setPower(pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(pow);
        } else if (buttonLeft1) {
            // slowly moves left
            leftFront.setPower(-pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(-pow);
        } else if (rb1){
            // rotate slowly right (clockwise)
            leftFront.setPower(pow);
            leftBack.setPower(pow);
            rightFront.setPower(-pow);
            rightBack.setPower(-pow);
        }
        else if (lb1) {
            // rotate slowly left (counter-clockwise)
            leftFront.setPower(-pow);
            leftBack.setPower(-pow);
            rightFront.setPower(pow);
            rightBack.setPower(pow);
        } else if (b1){
            shake = 1;
        } else if (shake > 6){
            shake = 0;
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        } else if (shake % 2 == 0 && shake > 0){
            leftFront.setPower(pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(pow);
            shake++;
        } else if (shake % 2 != 0 && shake > 0){
            leftFront.setPower(-pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(-pow);
            shake++;
        }else if (a1){
            shake2 = 1;
        } else if (shake2 > 6){
            shake2 = 0;
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        } else if (shake2 % 2 == 0 && shake2 > 0){
            leftFront.setPower(pow);
            leftBack.setPower(-pow);
            rightFront.setPower(-pow);
            rightBack.setPower(pow);
            shake++;
        } else if (shake2 % 2 != 0 && shake2 > 0){
            leftFront.setPower(-pow);
            leftBack.setPower(pow);
            rightFront.setPower(pow);
            rightBack.setPower(-pow);
            shake++;
        }

        // Arm code
        telemetry.addData("arm encoder", armEncoder.getCurrentPosition());
        if(a2) armPow = 1;
        else armPow = 0.9;
        armPos = armEncoder.getCurrentPosition();

        //reset encoder to zero positions based on zero is in bot
        if (back2){
            armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //set encoder to zero position
        }

        //set where we want to be
        if(a2){
            //manual
            desArmPos = 0;
        } else if (b2){
            desArmPos = -250; //wall
        }else if (y2){
            desArmPos = -3164; //basket
        } else if (x2){
            desArmPos = -3615; //bar
        }

        //arm moving stuff
        if (Math.abs(lefty2) >= .1 ) { //joystick
            armPow = 0.9;
            armLeft.setPower(lefty2*armPow);
            armRight.setPower(lefty2*armPow);
            desArmPos = 0;
        } else if (buttonUp2){ //manual slow
            armPow = 0.3;
            armLeft.setPower(armPow);
            armRight.setPower(armPow);
            desArmPos = 0;
        } else if (buttonDown2){ //manual slow
            armPow = 0.3;
            armLeft.setPower(-armPow);
            armRight.setPower(-armPow);
            desArmPos = 0;
        } else if (desArmPos != 0) { // encoder
            armPow = (armPos - desArmPos) * -0.0025;
            armLeft.setPower(armPow);
            armRight.setPower(armPow);
        } else { // turn off
            armLeft.setPower(0);
            armRight.setPower(0);
        }

        telemetry.addData("Desired Arm Positio", desArmPos);
        telemetry.addData("arm power", armPow);

        // intake code
        if (lb2){
            intake.setPower(0.5); // open
        }
        else if (rb2){
            intake.setPower(-0.5); // close
        } else{
            intake.setPower(0);
        }

        // emergency stop
        if (b1 && y1) {
            stop();
        }
    }

    public void stop() {
        // stop code
    }

}
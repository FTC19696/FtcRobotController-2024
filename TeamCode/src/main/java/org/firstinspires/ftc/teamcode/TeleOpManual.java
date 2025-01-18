package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp", group ="Linear Opmode" )
public class TeleOpManual extends LinearOpMode {
    @Override
    public void runOpMode(){
        // Declare driver and sensor inputs read from the joysticks and devices on the robot.
        double driverForward;
        double driverRotate;
        double driverStrafe;
        boolean driverSlow;
        double coDriverLift;
        double coDriverArm;
        boolean coDriverElbowUp;
        boolean coDriverElbowDown;
        boolean coDriverClawClose;
        boolean coDriverClawOpen;

        // Declare computed variables that will be sent to the robot objects or used
        // in further computation.
        double calcFrontLeft;
        double calcFrontRight;
        double calcBackLeft;
        double calcBackRight;
        double calcSpeedFactor;

        // Robot objects that will be used to send commands to motors, servos, and
        // other on-robot devices that can be controlled remotely.
        DcMotor robotFrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor robotFrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor robotBackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor robotBackRightMotor = hardwareMap.get(DcMotor.class, "BackRight");
        DcMotor robotLiftMotors = hardwareMap.get(DcMotor.class, "Lift");
        DcMotor robotElbowLeftMotor = hardwareMap.get(DcMotor.class, "ElbowLeft");
        DcMotor robotElbowRightMotor = hardwareMap.get(DcMotor.class, "ElbowRight");
        CRServo robotArmServo = hardwareMap.get(CRServo.class, "Arm");
        Servo robotClawLeftServo = hardwareMap.get(Servo.class, "ClawLeft");
        Servo robotClawRightServo = hardwareMap.get(Servo.class, "ClawRight");
        Servo robotDifferentialLeftServo = hardwareMap.get(Servo.class, "DifferentialLeft");
        Servo robotDifferentialRightServo = hardwareMap.get(Servo.class, "DifferentialRight");

        // Perform initialization of robot objects to make them ready to accept
        // commands once the robot becomes active (start pressed).
        robotFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotLiftMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotElbowLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotElbowRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robotFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotFrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotBackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotLiftMotors.setDirection(DcMotorSimple.Direction.FORWARD);
        robotElbowLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotElbowRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);



        // Robot is fully initialized and waiting for start button to be pressed
        // after autonomous is completed and teleop begins.
        waitForStart();

        robotClawLeftServo.setPosition(0.962);
        robotClawRightServo.setPosition(0.038);

        // Pre-position components to their initial location at the start of teleop


        // Main polling loop. Continue to loop through the sequence of reading,
        // computing, and transmitting commands back to the robot.
        while (opModeIsActive()) {
            // Reading the inputs.
            driverForward = -1 * gamepad1.left_stick_y;
            driverRotate = gamepad1.right_stick_x;
            driverStrafe = gamepad1.left_stick_x;
            driverSlow = gamepad1.right_bumper;
            coDriverLift = gamepad2.left_stick_y;
            coDriverClawClose = gamepad2.left_bumper;
            coDriverClawOpen = gamepad2.right_bumper;
            coDriverArm = gamepad2.right_stick_y;
            coDriverElbowUp = gamepad2.dpad_up;
            coDriverElbowDown = gamepad2.dpad_down;

            // computing the output values.
            if (driverSlow) {
                calcSpeedFactor = 0.3;
            } else {
                calcSpeedFactor = 1.0;
            }
            calcFrontLeft = (driverForward + driverRotate + driverStrafe) * calcSpeedFactor;
            calcFrontRight = (driverForward - driverRotate - driverStrafe) * calcSpeedFactor;
            calcBackLeft = (driverForward + driverRotate - driverStrafe) * calcSpeedFactor;
            calcBackRight = (driverForward - driverRotate + driverStrafe) * calcSpeedFactor;

            // Sending the power to the motors.
            robotFrontLeftMotor.setPower(calcFrontLeft);
            robotFrontRightMotor.setPower(calcFrontRight);
            robotBackLeftMotor.setPower(calcBackLeft);
            robotBackRightMotor.setPower(calcBackRight);
            robotLiftMotors.setPower(coDriverLift);

            // Lift motors.
            robotLiftMotors.setPower(coDriverLift);

            // Elbow motors.
            if (coDriverElbowUp) {
                robotElbowLeftMotor.setPower(-0.5);
                robotElbowRightMotor.setPower(-0.5);
            } else if (coDriverElbowDown) {
                robotElbowLeftMotor.setPower(0.4);
                robotElbowRightMotor.setPower(0.4);
            } else {
                robotElbowLeftMotor.setPower(0);
                robotElbowRightMotor.setPower(0);
            }

            // Arm extender.
            robotArmServo.setPower(coDriverArm);

            // Claw servos
            if (coDriverClawOpen) {
                robotClawLeftServo.setPosition(0.75);
                robotClawRightServo.setPosition(0.25);
            } else if(coDriverClawClose) {
                robotClawLeftServo.setPosition(0.962);
                robotClawRightServo.setPosition(0.038);
            }

            if (gamepad2.a){
                robotDifferentialLeftServo.setPosition(0.3);
                robotDifferentialRightServo.setPosition(0.7);
            }else if(gamepad2.y){
                robotDifferentialLeftServo.setPosition(0.4);
                robotDifferentialRightServo.setPosition(0.6);
            }else if (gamepad2.x){
                robotDifferentialLeftServo.setPosition(0);
                robotDifferentialRightServo.setPosition(1);

            }


            //Maybe toggle.
//            if (coDriverLeftDifferential) {
//                robotClawRightServo.setPosition(0.25);
//            } else {
//                robotClawRightServo.setPosition(0);
//            }
//            if (coDriverRightDifferential) {
//                robotClawRightServo.setPosition(0.75);
//            } else {
//                robotClawRightServo.setPosition(1);
//            }

            // Maybe toggle.
//            if (coDriverLeftClaw){
//                robotClawLeftServo.setPosition(-0.25);
//            }else {
//                robotClawLeftServo.setPosition(0);
//            }
//
//            if (coDriverRightClaw){
//                robotClawRightServo.setPosition(0.25);
//            }else{
//                robotClawRightServo.setPosition(0);
//            }
        }
    }
}
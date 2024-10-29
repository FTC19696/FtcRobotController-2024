package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp", group ="Linear Opmode" )
public class ManualTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare driver and sensor inputs read from the joysticks and devices on the robot.
        double driverForward;
        double driverRotate;
        double driverStrafe;
        boolean driverSlow;
        double coDriverLift;
        boolean coDriverLeftClaw;
        boolean coDriverRightClaw;


        // Declare computed variables that will be sent to the robot objects or used
        // in further computation.
        double calcFrontLeft;
        double calcFrontRight;
        double calcBackLeft;
        double calcBackRight;
        double calcSpeedFactor;

        // Robot objects that will be used to send commands to motors, servos, and
        // other on-robot devices that can be controlled remotely.
        DcMotor robotFrontLeftMotor;
        robotFrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");

        DcMotor robotFrontRightMotor;
        robotFrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");

        DcMotor robotBackLeftMotor;
        robotBackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");

        DcMotor robotBackRightMotor;
        robotBackRightMotor = hardwareMap.get(DcMotor.class, "BackRight");

        DcMotor robotLiftMotors;
        robotLiftMotors = hardwareMap.get(DcMotor.class, "LiftMotors");

        Servo robotClawServoLeft;
        robotClawServoLeft = hardwareMap.get(Servo.class, "ClawServoLeft");

        Servo robotClawServoRight;
        robotClawServoRight = hardwareMap.get(Servo.class, "ClawServoRight");

        // Perform initialization of robot objects to make them ready to accept
        // commands once the robot becomes active (start pressed).
        robotFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotLiftMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robotFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotFrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotBackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robotLiftMotors.setDirection(DcMotorSimple.Direction.REVERSE);

        robotClawServoLeft.setPosition(0);
        robotClawServoRight.setPosition(0);

        // Robot is fully initialized and waiting for start button to be pressed
        // after autonomous is completed and teleop begins.
        waitForStart();

        // Main polling loop. Continue to loop through the sequence of reading,
        // computing, and transmitting commands back to the robot.
        while (opModeIsActive()) {
            // Reading the inputs.
            driverForward = -1 * gamepad1.left_stick_y;
            driverRotate = gamepad1.right_stick_x;
            driverStrafe = gamepad1.left_stick_x;
            driverSlow = gamepad1.right_bumper;
            coDriverLift = gamepad2.left_stick_y;
            coDriverLeftClaw = gamepad2.left_bumper;
            coDriverRightClaw = gamepad2.right_bumper;

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

            // Sending the power to the servos
            if (coDriverLeftClaw){
                robotClawServoLeft.setPosition(-0.25);
            }else {
                robotClawServoLeft.setPosition(0);
            }

            if (coDriverRightClaw){
                robotClawServoRight.setPosition(0.25);
            }else{
                robotClawServoRight.setPosition(0);
            }
        }
    }
}
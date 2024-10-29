package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueAutoObv", group = "Robot")
public class BlueAutoObv extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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

            // computing the output values.

            // Sending the power to the motors.
        }
    }
}
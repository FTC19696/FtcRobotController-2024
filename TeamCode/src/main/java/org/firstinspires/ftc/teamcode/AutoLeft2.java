package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoLeft2", group = "Robot")
public class AutoLeft2 extends LinearOpMode {
    private static final double TICKS_PER_CM = 12.8;

    @Override
    public void runOpMode() {
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

        // Perform initialization of robot objects to make them ready to accept
        // commands once the robot becomes active (start pressed).
        configureMotors(robotFrontLeftMotor, robotFrontRightMotor, robotBackLeftMotor, robotBackRightMotor);

        // Robot is fully initialized and waiting for start button to be pressed
        waitForStart();

        robotClawLeftServo.setPosition(0);
        robotClawRightServo.setPosition(0);

        driveForward(85, 0.5, robotFrontLeftMotor, robotFrontRightMotor, robotBackLeftMotor, robotBackRightMotor);

        // Robot strafes left.
        strafeLeft(97, 0.5, robotFrontLeftMotor, robotFrontRightMotor, robotBackLeftMotor, robotBackRightMotor);

        // Robot goes forward.
        driveForward(90, 0.5, robotFrontLeftMotor, robotFrontRightMotor, robotBackLeftMotor, robotBackRightMotor);

        // Robot turns.
        rotateRight(85, 0.5, robotFrontLeftMotor, robotFrontRightMotor, robotBackLeftMotor, robotBackRightMotor);

        // Robot goes forward.
        driveForward(30, 0.5, robotFrontLeftMotor, robotFrontRightMotor, robotBackLeftMotor, robotBackRightMotor);
    }

    private void driveForward(int distanceCentimeters, double power, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        stop(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        frontLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        frontRightMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backRightMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));

        move(power, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    private void strafeLeft(int distanceCentimeters, double power, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        stop(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        frontLeftMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));
        frontRightMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backRightMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));

        move(power, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    private void strafeRight(int distanceCentimeters, double power, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        stop(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        frontLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        frontRightMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));
        backLeftMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));
        backRightMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));

        move(power, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    private void rotateRight(int distanceCentimeters, double power, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        stop(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        frontLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        frontRightMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));
        backLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backRightMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));

        move(power, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    private void configureMotors(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stop(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void move(double power, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);

        while (frontLeftMotor.isBusy() | frontRightMotor.isBusy() | backLeftMotor.isBusy() | backRightMotor.isBusy()) {
            sleep(10);
        }
    }
}
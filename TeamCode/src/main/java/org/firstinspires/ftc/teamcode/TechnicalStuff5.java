package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TechnicalStuff5 {
    private static final double TICKS_PER_CM = 12.8;

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor liftMotors;
    public DcMotor elbowLeftMotor;
    public DcMotor elbowRightMotor;
    public CRServo armServo;
    public Servo clawLeftServo;
    public Servo clawRightServo;

    public TechnicalStuff5(HardwareMap hardwareMap){
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRight");
        liftMotors = hardwareMap.get(DcMotor.class, "Lift");
        elbowLeftMotor = hardwareMap.get(DcMotor.class, "ElbowLeft");
        elbowRightMotor = hardwareMap.get(DcMotor.class, "ElbowRight");
        armServo = hardwareMap.get(CRServo.class, "Arm");
        clawLeftServo = hardwareMap.get(Servo.class, "ClawLeft");
        clawRightServo = hardwareMap.get(Servo.class, "ClawRight");

        configureMotors();
    }

    public void driveForward(int distanceCentimeters, double power) {
        stop();

        frontLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        frontRightMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backRightMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));

        moveToTarget(power);
    }

    public void strafeLeft(int distanceCentimeters, double power) {
        stop();

        frontLeftMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));
        frontRightMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backRightMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));

        moveToTarget(power);
    }

    public void strafeRight(int distanceCentimeters, double power) {
        stop();

        frontLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        frontRightMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));
        backLeftMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));
        backRightMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));

        moveToTarget(power);
    }

    public void rotateRight(int distanceCentimeters, double power) {
        stop();

        frontLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        frontRightMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));
        backLeftMotor.setTargetPosition((int) (distanceCentimeters * TICKS_PER_CM));
        backRightMotor.setTargetPosition((int) (-1 * distanceCentimeters * TICKS_PER_CM));

        moveToTarget(power);
    }

    private void configureMotors() {
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

    private void stop() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveToTarget(double power) {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);

        while (frontLeftMotor.isBusy() | frontRightMotor.isBusy() | backLeftMotor.isBusy() | backRightMotor.isBusy()) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
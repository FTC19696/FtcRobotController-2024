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

    public TechnicalStuff5(HardwareMap hardwareMap) {
        if (hardwareMap == null)
            throw new IllegalArgumentException("Attempted to construct TechnicalStuff5 object using" +
                    "an uninitialized HardwareMap. Make sure to call this constructor only from" +
                    "an OpMode after initialization (ex. from inside the runOpMode() method).");

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

    public void driveParametric(double forward, double rotate, double strafe, boolean slow) {
        double calcFrontLeft;
        double calcFrontRight;
        double calcBackLeft;
        double calcBackRight;
        double calcSpeedFactor;

        if (slow) {
            calcSpeedFactor = 0.3;
        } else {
            calcSpeedFactor = 1.0;
        }
        calcFrontLeft = (forward + rotate + strafe) * calcSpeedFactor;
        calcFrontRight = (forward - rotate - strafe) * calcSpeedFactor;
        calcBackLeft = (forward + rotate - strafe) * calcSpeedFactor;
        calcBackRight = (forward - rotate + strafe) * calcSpeedFactor;

        // Send power to drive motors.
        frontLeftMotor.setPower(calcFrontLeft);
        frontRightMotor.setPower(calcFrontRight);
        backLeftMotor.setPower(calcBackLeft);
        backRightMotor.setPower(calcBackRight);
    }

    public void driveForward(int distanceCentimeters, double power) {
        driveStop();

        driveToProportionalTarget(power,
                (int) (distanceCentimeters * TICKS_PER_CM),
                (int) (distanceCentimeters * TICKS_PER_CM),
                (int) (distanceCentimeters * TICKS_PER_CM),
                (int) (distanceCentimeters * TICKS_PER_CM));
    }

    public void strafeRight(int distanceCentimeters, double power) {
        driveStop();

        driveToProportionalTarget(power,
                (int) (distanceCentimeters * TICKS_PER_CM),
                (int) (-1 * distanceCentimeters * TICKS_PER_CM),
                (int) (-1 * distanceCentimeters * TICKS_PER_CM),
                (int) (distanceCentimeters * TICKS_PER_CM));
    }

    public void rotateRight(int distanceCentimeters, double power) {
        driveStop();

        driveToProportionalTarget(power,
                (int) (distanceCentimeters * TICKS_PER_CM),
                (int) (-1 * distanceCentimeters * TICKS_PER_CM),
                (int) (distanceCentimeters * TICKS_PER_CM),
                (int) (-1 * distanceCentimeters * TICKS_PER_CM));
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
        liftMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotors.setDirection(DcMotorSimple.Direction.FORWARD);
        elbowLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveStop() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void driveToProportionalTarget(double power, int frontLeft, int frontRight, int backLeft, int backRight) {
        frontLeftMotor.setTargetPosition(frontLeft);
        frontRightMotor.setTargetPosition(frontRight);
        backLeftMotor.setTargetPosition(backLeft);
        backRightMotor.setTargetPosition(backRight);

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
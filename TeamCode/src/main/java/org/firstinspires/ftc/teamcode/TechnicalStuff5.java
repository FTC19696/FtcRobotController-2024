package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TechnicalStuff5 {
    public Chassis chassis;
    public Lift lift;
    public Elbow elbow;
    public Arm arm;
    public Claw claw;

    public TechnicalStuff5(HardwareMap hardwareMap) {
        if (hardwareMap == null)
            throw new IllegalArgumentException("Attempted to construct TechnicalStuff5 object using" +
                    "an uninitialized HardwareMap. Make sure to call this constructor only from" +
                    "an OpMode after initialization (ex. from inside the runOpMode() method).");

        chassis = new Chassis(hardwareMap);
        lift = new Lift(hardwareMap);
        elbow = new Elbow(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    // Nested Classes

    // region Chassis Class

    public static class Chassis {
        private static final double TICKS_PER_CM = 12.8;

        private final DcMotor frontLeftMotor;
        private final DcMotor frontRightMotor;
        private final DcMotor backLeftMotor;
        private final DcMotor backRightMotor;

        public Chassis(HardwareMap hardwareMap) {
            if (hardwareMap == null)
                throw new IllegalArgumentException("Attempted to construct Chassis object using" +
                        "an uninitialized HardwareMap. Make sure to call this constructor only from" +
                        "an OpMode after initialization (ex. from inside the runOpMode() method).");

            frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
            frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
            backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
            backRightMotor = hardwareMap.get(DcMotor.class, "BackRight");

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

        public void rotateClockwise(int distanceCentimeters, double power) {
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

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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

    //endregion

    // region Lift Class

    public class Lift {
        private final DcMotor motors;

        public Lift(HardwareMap hardwareMap) {
            if (hardwareMap == null)
                throw new IllegalArgumentException("Attempted to construct Lift object using" +
                        "an uninitialized HardwareMap. Make sure to call this constructor only from" +
                        "an OpMode after initialization (ex. from inside the runOpMode() method).");

            chassis = new Chassis(hardwareMap);

            motors = hardwareMap.get(DcMotor.class, "Lift");

            configureMotors();
        }

        public void raise(double power) {
            motors.setPower(power);
        }

        private void configureMotors() {
            motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motors.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    // endregion

    // region Elbow Class

    public static class Elbow {
        private final DcMotor leftMotor;
        private final DcMotor rightMotor;

        public Elbow(HardwareMap hardwareMap) {
            if (hardwareMap == null)
                throw new IllegalArgumentException("Attempted to construct Elbow object using" +
                        "an uninitialized HardwareMap. Make sure to call this constructor only from" +
                        "an OpMode after initialization (ex. from inside the runOpMode() method).");

            leftMotor = hardwareMap.get(DcMotor.class, "ElbowLeft");
            rightMotor = hardwareMap.get(DcMotor.class, "ElbowRight");

            configureMotors();
        }

        public void raise(double power) {
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }

        private void configureMotors() {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    //endregion

    // region Arm Class

    public static class Arm {
        private final CRServo extensionServo;

        public Arm(HardwareMap hardwareMap) {
            if (hardwareMap == null)
                throw new IllegalArgumentException("Attempted to construct Arm object using" +
                        "an uninitialized HardwareMap. Make sure to call this constructor only from" +
                        "an OpMode after initialization (ex. from inside the runOpMode() method).");

            extensionServo = hardwareMap.get(CRServo.class, "Arm");
        }

        public void extend(double power) {
            extensionServo.setPower(power);
        }
    }

    // endregion

    // region Claw Class

    public static class Claw {
        private final Servo leftServo;
        private final Servo rightServo;

        public Claw(HardwareMap hardwareMap) {
            if (hardwareMap == null)
                throw new IllegalArgumentException("Attempted to construct Claw object using" +
                        "an uninitialized HardwareMap. Make sure to call this constructor only from" +
                        "an OpMode after initialization (ex. from inside the runOpMode() method).");

            leftServo = hardwareMap.get(Servo.class, "ClawLeft");
            rightServo = hardwareMap.get(Servo.class, "ClawRight");
        }

        public void setPosition(double left, double right){
            leftServo.setPosition(left);
            rightServo.setPosition(right);
        }
    }

    // endregion
}
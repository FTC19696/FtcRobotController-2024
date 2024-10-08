package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Locale;

public class TSStudicabot {
    private final DriveBase driveBase;
//    private Pylon pylon;
//    private Arm arm;

    public TSStudicabot(HardwareMap hardwareMap) {
        driveBase = new DriveBase(hardwareMap);
//        arm = new Arm(hardwareMap);
//        pylon = new Pylon(hardwareMap);
    }

    public void sendCommands(RobotCommands robotCommands) {
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        frontLeftPower = robotCommands.moveForward + robotCommands.moveRotate + robotCommands.moveLateral;
        frontRightPower = robotCommands.moveForward - robotCommands.moveRotate - robotCommands.moveLateral;
        backLeftPower = robotCommands.moveForward + robotCommands.moveRotate - robotCommands.moveLateral;
        backRightPower = robotCommands.moveForward - robotCommands.moveRotate + robotCommands.moveLateral;

        this.driveBase.setPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
//        this.pylon.setPower(robotCommands.spinPylon);
    }

    public String getTelemetryReport() {
        return String.format(Locale.US,
                "left ( %.2f), right (%.2f)", this.driveBase.frontLeftMotor.getPower(),
                this.driveBase.frontRightMotor.getPower());
    }

    private static class DriveBase {
        private DcMotor frontLeftMotor = null;
        private DcMotor frontRightMotor = null;
        private DcMotor backLeftMotor = null;
        private DcMotor backRightMotor = null;

        public DriveBase(HardwareMap hardwareMap) {
            frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
            frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
            backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
            backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        public void setPower(double frontLeft, double frontRight, double backLeft, double backRight) {
            this.frontLeftMotor.setPower(frontLeft);
            this.frontRightMotor.setPower(frontRight);
            this.backLeftMotor.setPower(backLeft);
            this.backRightMotor.setPower(backRight);
        }

//        public void stop() {
//            this.frontRightMotor.setPower(0);
//            this.frontLeftMotor.setPower(0);
//            this.backRightMotor.setPower(0);
//            this.backLeftMotor.setPower(0);
//        }
    }

//    private static class Pylon {
//        private DcMotor spiralMotor = null;
//
//        public Pylon(HardwareMap hardwareMap) {
//            spiralMotor = hardwareMap.get(DcMotor.class, "Spiral");
//            spiralMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            spiralMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//
//        public void setPower(double power) {
//            spiralMotor.setPower(power);
//        }
//    }
//
//    private static class Arm {
//        public Arm(HardwareMap hardwareMap) {
//        }
//    }
}
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Locale;

public class TSStudicabot {
    private final DriveBase driveBase;

    public TSStudicabot(HardwareMap hardwareMap) {
        driveBase = new DriveBase(hardwareMap);
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
    }

    public String getTelemetryReport() {
        return String.format(Locale.US,
                "FL %.2f, FR %.2f, BL %.2f, BR %.2f",
                this.driveBase.frontLeftMotor.getPower(),
                this.driveBase.frontRightMotor.getPower(),
                this.driveBase.backLeftMotor.getPower(),
                this.driveBase.backRightMotor.getPower());
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
    }
}
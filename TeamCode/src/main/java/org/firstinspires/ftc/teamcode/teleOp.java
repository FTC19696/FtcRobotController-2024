package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "teleOp", group ="Liner Opmode" )
public class teleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // The joystick inputs.
        double driverForward;
        double driverRotate;
        double driverStrafe;
        boolean driverSlow;
        // These are the computed variables that will be sent to the robot.
        double calcFrontLeft;
        double calcFrontRight;
        double calcBackLeft;
        double calcBackRight;
        double calcSpeedFactor;

        // Declaring the motors.
        DcMotor robotObjectsFrontLeftMotor;
        robotObjectsFrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        robotObjectsFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotObjectsFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor robotObjectsFrontRightMotor;
        robotObjectsFrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        robotObjectsFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotObjectsFrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor robotObjectsBackLeftMotor;
        robotObjectsBackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        robotObjectsBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotObjectsBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor robotObjectsBackRightMotor;
        robotObjectsBackRightMotor = hardwareMap.get(DcMotor.class, "BackRight");
        robotObjectsBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotObjectsBackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();
        while(opModeIsActive()){
        // Reading the inputs.
            driverForward = -1*gamepad1.left_stick_y;
            driverRotate = gamepad1.right_stick_x;
            driverStrafe = gamepad1.left_stick_x;
            driverSlow = gamepad1.right_bumper;

            // computing the output values.
            if (driverSlow){
                calcSpeedFactor = 0.3;
            }else{
                calcSpeedFactor = 1.0;
            }
            calcFrontLeft = (driverForward + driverRotate + driverStrafe)*calcSpeedFactor;
            calcFrontRight = (driverForward - driverRotate - driverStrafe)*calcSpeedFactor;
            calcBackLeft = (driverForward + driverRotate - driverStrafe)*calcSpeedFactor;
            calcBackRight = (driverForward - driverRotate + driverStrafe)*calcSpeedFactor;

            // Sending the power to the motors.
            robotObjectsFrontLeftMotor.setPower(calcFrontLeft);
            robotObjectsFrontRightMotor.setPower(calcFrontRight);
            robotObjectsBackLeftMotor.setPower(calcBackLeft);
            robotObjectsBackRightMotor.setPower(calcBackRight);



        }
    }
}

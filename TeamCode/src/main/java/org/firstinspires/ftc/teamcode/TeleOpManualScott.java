package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp-Scott", group="Sample" )
public class TeleOpManualScott extends LinearOpMode {
    @Override
    public void runOpMode() {
        TechnicalStuff5 ts5 = new TechnicalStuff5(hardwareMap);

        // Declare driver and sensor inputs read from the joysticks and devices on the robot.
        double driverForward;
        double driverRotate;
        double driverStrafe;
        boolean driverSlow;
        double coDriverLift;
        double coDriverArm;
        boolean coDriverElbowUp;
        boolean coDriverElbowDown;
        boolean coDriverLeftClaw;
        boolean coDriverRightClaw;

        // Robot is fully initialized and waiting for start button to be pressed
        // after autonomous is completed and teleop begins.
        waitForStart();

        // Pre-position components to their initial location at the start of teleop
        ts5.clawLeftServo.setPosition(0);
        ts5.clawRightServo.setPosition(0);

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
            coDriverArm = gamepad2.right_stick_y;
            coDriverElbowUp = gamepad2.dpad_up;
            coDriverElbowDown = gamepad2.dpad_down;

            // Chassis drive motors.
            ts5.driveParametric(driverForward, driverRotate, driverStrafe, driverSlow);

            // Lift motors.
            ts5.liftMotors.setPower(coDriverLift);

            // Elbow motors.
            if (coDriverElbowUp) {
                ts5.elbowLeftMotor.setPower(1);
                ts5.elbowRightMotor.setPower(1);
            } else if (coDriverElbowDown) {
                ts5.elbowLeftMotor.setPower(-0.25);
                ts5.elbowRightMotor.setPower(-0.25);
            } else {
                ts5.elbowLeftMotor.setPower(0);
                ts5.elbowRightMotor.setPower(0);
            }

            // Arm extender.
            ts5.armServo.setPower(coDriverArm);

            // Claw servos
            if (coDriverLeftClaw) {
                ts5.clawLeftServo.setPosition(0.75);
            } else {
                ts5.clawLeftServo.setPosition(1);
            }
            if (coDriverRightClaw) {
                ts5.clawRightServo.setPosition(0.25);
            } else {
                ts5.clawRightServo.setPosition(0);
            }
        }
    }
}
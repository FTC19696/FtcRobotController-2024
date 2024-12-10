package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp-Scott", group="Sample" )
public class TeleOpManualScott extends LinearOpMode {
    @Override
    public void runOpMode() {
        TechnicalStuff5 ts5 = new TechnicalStuff5(hardwareMap);

        // Declare driver and sensor inputs read from the joysticks and sensors on the robot.
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
        ts5.claw.leftServo.setPosition(0);
        ts5.claw.rightServo.setPosition(0);

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

            // Chassis drive motors
            ts5.chassis.driveParametric(driverForward, driverRotate, driverStrafe, driverSlow);

            // Lift motors
            ts5.lift.motors.setPower(coDriverLift);

            // Elbow motors
            if (coDriverElbowUp) {
                ts5.elbow.leftMotor.setPower(1);
                ts5.elbow.rightMotor.setPower(1);
            } else if (coDriverElbowDown) {
                ts5.elbow.leftMotor.setPower(-0.25);
                ts5.elbow.rightMotor.setPower(-0.25);
            } else {
                ts5.elbow.leftMotor.setPower(0);
                ts5.elbow.rightMotor.setPower(0);
            }

            // Arm extender
            ts5.arm.extensionServo.setPower(coDriverArm);

            // Claw servos
            if (coDriverLeftClaw) {
                ts5.claw.leftServo.setPosition(0.75);
            } else {
                ts5.claw.leftServo.setPosition(1);
            }
            if (coDriverRightClaw) {
                ts5.claw.rightServo.setPosition(0.25);
            } else {
                ts5.claw.rightServo.setPosition(0);
            }
        }
    }
}
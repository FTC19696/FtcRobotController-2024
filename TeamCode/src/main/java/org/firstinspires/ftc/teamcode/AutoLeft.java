package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoLeft", group = "Robot")
public class AutoLeft extends LinearOpMode {
    private static final double TICKS_PER_CM = 12.8;

    @Override
    public void runOpMode() throws InterruptedException {
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
        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        robotElbowLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robotElbowRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLiftMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotElbowLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotElbowRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Robot is fully initialized and waiting for start button to be pressed
        waitForStart();

        // lift goes up.
        robotLiftMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotLiftMotors.setTargetPosition((int) (80 * TICKS_PER_CM));

        robotLiftMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robotLiftMotors.setPower(1);

        while (robotLiftMotors.isBusy()) {
        }

        sleep(250);

        robotClawLeftServo.setPosition(0.6);
        robotClawRightServo.setPosition(0.4);

        sleep(250);

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotFrontLeftMotor.setTargetPosition((int) (85 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (85 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (85 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (85 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Robot move to bar.
        robotFrontLeftMotor.setPower(0.5);
        robotFrontRightMotor.setPower(0.5);
        robotBackLeftMotor.setPower(0.5);
        robotBackRightMotor.setPower(0.5);
        while (robotFrontLeftMotor.isBusy()) {
        }
        sleep(250);

        // Robot strafes left.
        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotFrontLeftMotor.setTargetPosition((int) (-97 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (97 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (97 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (-97 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robotFrontLeftMotor.isBusy()) {
        }
        sleep(250);

        // Robot goes forward.
        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotFrontLeftMotor.setTargetPosition((int) (90 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (90 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (90 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (90 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robotFrontLeftMotor.isBusy()) {
        }
        sleep(250);

        // Robot turns.
        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotFrontLeftMotor.setTargetPosition((int) (85 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (-85 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (85 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (-85 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robotFrontLeftMotor.isBusy()) {
        }
        sleep(250);

        //arm goes up.
        robotElbowLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotElbowRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotElbowLeftMotor.setTargetPosition((int) (20 * TICKS_PER_CM));
        robotElbowRightMotor.setTargetPosition((int) (20 * TICKS_PER_CM));

        robotElbowLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotElbowRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robotElbowLeftMotor.setPower(1);
        robotElbowRightMotor.setPower(1);

        while (robotElbowLeftMotor.isBusy()) {
        }


        // Robot goes forward.
        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotFrontLeftMotor.setTargetPosition((int) (30 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (30 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (30 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (30 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robotFrontLeftMotor.isBusy()) {
        }
        sleep(250);
    }
}
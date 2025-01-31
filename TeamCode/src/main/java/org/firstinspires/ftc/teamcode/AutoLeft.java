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
        Servo robotDifferentialLeftServo = hardwareMap.get(Servo.class, "DifferentialLeft");
        Servo robotDifferentialRightServo = hardwareMap.get(Servo.class, "DifferentialRight");

        // Perform initialization of robot objects to make them ready to accept
        // commands once the robot becomes active (start pressed)
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

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotLiftMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Robot is fully initialized and waiting for start button to be pressed
        waitForStart();

        robotClawLeftServo.setPosition(0.962);
        robotClawRightServo.setPosition(0.038);
        robotDifferentialLeftServo.setPosition(0.3);
        robotDifferentialRightServo.setPosition(0.7);


        // Robot move to bar.
        robotFrontLeftMotor.setTargetPosition((int) (85 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (85 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (85 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (85 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);
        //Robot moves left.
        robotFrontLeftMotor.setTargetPosition((int) (-100 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (100 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (100 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (-100 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.5);
        robotFrontRightMotor.setPower(0.5);
        robotBackLeftMotor.setPower(0.5);
        robotBackRightMotor.setPower(0.5);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);
        //Robot move forwards.
        robotFrontLeftMotor.setTargetPosition((int) (110 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (110 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (110 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (110 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot does a 180.
        robotFrontLeftMotor.setTargetPosition((int) (-175 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (175 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (-175 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (175 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.5);
        robotFrontRightMotor.setPower(0.5);
        robotBackLeftMotor.setPower(0.5);
        robotBackRightMotor.setPower(0.5);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot moves right.
        robotFrontLeftMotor.setTargetPosition((int) (45 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (-45 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (-45 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (45 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.5);
        robotFrontRightMotor.setPower(0.5);
        robotBackLeftMotor.setPower(0.5);
        robotBackRightMotor.setPower(0.5);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot move forwards.
        robotFrontLeftMotor.setTargetPosition((int) (95 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (95 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (95 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (95 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot turns right
        robotFrontLeftMotor.setTargetPosition((int) (44 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (-44 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (44 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (-44 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot move forwards.
        robotFrontLeftMotor.setTargetPosition((int) (80 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (80 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (80 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (80 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot move backwards.
        robotFrontLeftMotor.setTargetPosition((int) (-95 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (-95 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (-95 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (-95 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot turns left
        robotFrontLeftMotor.setTargetPosition((int) (-44 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (44 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (-44 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (44 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot move backwards.
        robotFrontLeftMotor.setTargetPosition((int) (-95 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (-95 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (-95 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (-95 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot moves right.
        robotFrontLeftMotor.setTargetPosition((int) (45 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (-45 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (-45 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (45 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.5);
        robotFrontRightMotor.setPower(0.5);
        robotBackLeftMotor.setPower(0.5);
        robotBackRightMotor.setPower(0.5);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot move forwards.
        robotFrontLeftMotor.setTargetPosition((int) (210 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (210 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (210 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (210 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot move backwards.
        robotFrontLeftMotor.setTargetPosition((int) (-210 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (-210 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (-210 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (-210 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot turns left
        robotFrontLeftMotor.setTargetPosition((int) (-88 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (88 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (-88 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (88 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        //Robot move forwards.
        robotFrontLeftMotor.setTargetPosition((int) (90 * TICKS_PER_CM));
        robotFrontRightMotor.setTargetPosition((int) (90 * TICKS_PER_CM));
        robotBackLeftMotor.setTargetPosition((int) (90 * TICKS_PER_CM));
        robotBackRightMotor.setTargetPosition((int) (90 * TICKS_PER_CM));

        robotFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robotFrontLeftMotor.setPower(0.75);
        robotFrontRightMotor.setPower(0.75);
        robotBackLeftMotor.setPower(0.75);
        robotBackRightMotor.setPower(0.75);
        while (robotFrontLeftMotor.isBusy()) {
        }

        robotFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);



//
//        // To do:
//        // Arm move up to bar.
//        // Arm move down to hook the hook.
//
//        //Claw lets go.
//
//        // Robot moves to the Obv.

        while (robotFrontLeftMotor.isBusy()) {
        }
    }
}
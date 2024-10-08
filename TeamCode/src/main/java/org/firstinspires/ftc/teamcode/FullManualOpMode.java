package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.driver.HumanDriverPair;
import org.firstinspires.ftc.teamcode.driver.DriverInputs;
import org.firstinspires.ftc.teamcode.robot.PredefinedRobotCommands;
import org.firstinspires.ftc.teamcode.robot.TSStudicabot;
import org.firstinspires.ftc.teamcode.robot.RobotCommands;

@TeleOp(name="Full Manual", group="Linear Opmode")
public class FullManualOpMode extends LinearOpMode {
    private final HumanDriverPair driveTeam;
    private final TSStudicabot robot;

    public FullManualOpMode() {
        this.driveTeam = new HumanDriverPair(this.gamepad1, this.gamepad2);
        this.robot = new TSStudicabot(hardwareMap);
    }

    @Override
    public void runOpMode() {
        startTelemetry();
        this.waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Read driver inputs from controllers
            DriverInputs driverInputs = driveTeam.getDriverInputs();

            // Translate driver inputs to robot commands
            RobotCommands robotCommands = mapRobotCommands(driverInputs);

            // Send commands to robot
            this.robot.sendCommands(robotCommands);

            // Show the elapsed game time and wheel power.
            writeTelemetry(this.robot.getTelemetryReport());
        }

        this.robot.sendCommands(PredefinedRobotCommands.fullStop());
    }

    private RobotCommands mapRobotCommands(DriverInputs driverInputs) {
        RobotCommands commands = new RobotCommands();

        commands.moveForward = driverInputs.forwardMotion;
        commands.moveRotate = driverInputs.rotateMotion;
        commands.moveLateral = driverInputs.strafeMotion;
        commands.spinPylon = driverInputs.pylonSpin;
        return commands;
    }

    private void startTelemetry() {
        this.telemetry.addData("Status", "Initialized");
        this.telemetry.update();
    }

    private void writeTelemetry(String report) {
        this.telemetry.addData("Motors", report);
        this.telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.driver.HumanDriverPair;
import org.firstinspires.ftc.teamcode.driver.DriverInputs;
import org.firstinspires.ftc.teamcode.robot.PredefinedRobotCommands;
import org.firstinspires.ftc.teamcode.robot.TSStudicabot;
import org.firstinspires.ftc.teamcode.robot.RobotCommands;

@TeleOp(name="Full Manual", group="Linear Opmode")
public class FullManualOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        writeHardwareMap(this.hardwareMap);

        HumanDriverPair driveTeam = new HumanDriverPair(this.gamepad1, this.gamepad2);
        TSStudicabot robot = new TSStudicabot(this.hardwareMap);

        startTelemetry();
        this.waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Read driver inputs from controllers
            DriverInputs driverInputs = driveTeam.getDriverInputs();

            // Show the elapsed game time and wheel power.
            writeTelemetryItem("Joy", driverInputs.getTelemetryReport());

            // Translate driver inputs to robot commands
            RobotCommands robotCommands = mapRobotCommands(driverInputs);

            // Send commands to robot
            robot.sendCommands(robotCommands);

            // Show the elapsed game time and wheel power.
            writeTelemetryItem("Mot", robot.getTelemetryReport());

            checkpointTelemetry();
        }

        robot.sendCommands(PredefinedRobotCommands.fullStop());
    }

    private RobotCommands mapRobotCommands(DriverInputs driverInputs) {
        RobotCommands commands = new RobotCommands();

        commands.moveForward = driverInputs.forwardMotion;
        commands.moveRotate = driverInputs.rotateMotion;
        commands.moveLateral = driverInputs.strafeMotion;
        commands.spinPylon = driverInputs.pylonSpin;
        return commands;
    }

    private void writeHardwareMap(HardwareMap hwMap)
    {
        if (hwMap == null)
            throw new IllegalArgumentException("HardwareMap is null.");

        for (DcMotor dcm : hwMap.dcMotor)
        {
            this.telemetry.addLine(dcm.toString());
        }
        telemetry.update();
    }

    private void startTelemetry() {
        this.telemetry.clearAll();
        this.telemetry.addData("Status", "Initialized");
        this.telemetry.update();
    }

    private void writeTelemetryItem(String name, String report) {
        this.telemetry.addData(name, report);
    }

    private void checkpointTelemetry()
    {
        this.telemetry.update();
    }
}
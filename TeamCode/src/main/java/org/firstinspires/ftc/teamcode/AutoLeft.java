package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeft", group = "Sample")
public class AutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        TechnicalStuff5 ts5 = new TechnicalStuff5(hardwareMap);

        // Robot is fully initialized and waiting for start button to be pressed
        waitForStart();

        ts5.claw.setPosition(0, 0);

        ts5.chassis.driveForward(85, 0.5);

        // Robot strafes left.
        ts5.chassis.strafeRight(-97, 0.5);

        // Robot goes forward.
        ts5.chassis.driveForward(90, 0.5);

        // Robot turns.
        ts5.chassis.rotateClockwise(85, 0.5);

        // Robot goes forward.
        ts5.chassis.driveForward(30, 0.5);
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeft2", group = "Robot")
public class AutoLeft2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        TechnicalStuff5 ts5 = new TechnicalStuff5(hardwareMap);

        // Robot is fully initialized and waiting for start button to be pressed
        waitForStart();

        ts5.clawLeftServo.setPosition(0);
        ts5.clawRightServo.setPosition(0);

        ts5.driveForward(85, 0.5);

        // Robot strafes left.
        ts5.strafeLeft(97, 0.5);

        // Robot goes forward.
        ts5.driveForward(90, 0.5);

        // Robot turns.
        ts5.rotateRight(85, 0.5);

        // Robot goes forward.
        ts5.driveForward(30, 0.5);
    }
}
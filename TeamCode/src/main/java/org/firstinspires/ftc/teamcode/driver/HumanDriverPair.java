package org.firstinspires.ftc.teamcode.driver;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Locale;

public class HumanDriverPair {
    private final Gamepad chassisGamepad;
    private final Gamepad torsoGamepad;

    public HumanDriverPair(Gamepad chassis, Gamepad torso) {
        chassisGamepad = chassis;
        torsoGamepad = torso;
    }

    public DriverInputs getDriverInputs() {
        DriverInputs result = new DriverInputs();

        result.forwardMotion = chassisGamepad.left_stick_y;
        result.rotateMotion = chassisGamepad.left_stick_x;
        result.strafeMotion = chassisGamepad.right_stick_x;
        result.pylonSpin = torsoGamepad.left_stick_x;
        return result;
    }
}
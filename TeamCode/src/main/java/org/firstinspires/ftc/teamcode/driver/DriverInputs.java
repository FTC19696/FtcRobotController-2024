package org.firstinspires.ftc.teamcode.driver;

import java.util.Locale;

public class DriverInputs {
    public double forwardMotion;
    public double rotateMotion;
    public double strafeMotion;
    public double pylonSpin;

    public String getTelemetryReport() {
        return String.format(Locale.US,
                "FWD %.2f, ROT %.2f, STR %.2f",
                forwardMotion,
                rotateMotion,
                strafeMotion);
    }
}
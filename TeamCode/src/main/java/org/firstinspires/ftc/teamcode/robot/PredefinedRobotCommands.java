package org.firstinspires.ftc.teamcode.robot;

public class PredefinedRobotCommands {
    public static RobotCommands fullStop(){
        RobotCommands commands = new RobotCommands();

        commands.moveForward = 0;
        commands.moveRotate = 0;
        commands.moveLateral = 0;
        commands.spinPylon = 0;
        return commands;
    }
}
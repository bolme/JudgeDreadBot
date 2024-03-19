package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;


public class ToggleFieldOriented extends InstantCommand {


    public ToggleFieldOriented() {
    }

    @Override
    public void initialize() {
        TeleopSwerveControl.getInstance().toggleFieldOriented();
    }
}
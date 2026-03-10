package frc.robot.commands.Lights.WPIlib;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib.LEDTarget;

public class ResetLED extends Command {
    private final LEDSubsystem_WPIlib ledSubsystem;
    private final LEDTarget target;

    public ResetLED(LEDSubsystem_WPIlib subsystem, LEDTarget target) {
        this.ledSubsystem = subsystem;
        this.target = target;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Reset(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
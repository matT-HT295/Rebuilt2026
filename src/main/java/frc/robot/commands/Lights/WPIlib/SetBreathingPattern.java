package frc.robot.commands.Lights.WPIlib;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import edu.wpi.first.wpilibj.LEDPattern;

public class SetBreathingPattern extends Command {
    private final LEDSubsystem_WPIlib ledSubsystem;
    private final LEDSubsystem_WPIlib.LEDTarget target;
    private final LEDPattern pattern;
    private final double period;

    public SetBreathingPattern(LEDSubsystem_WPIlib subsystem, LEDSubsystem_WPIlib.LEDTarget target, LEDPattern pattern, double period) {
        this.ledSubsystem = subsystem;
        this.target = target;
        this.pattern = pattern;
        this.period = period;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Breathing(target, pattern, period);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
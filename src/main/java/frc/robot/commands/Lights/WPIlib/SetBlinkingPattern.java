package frc.robot.commands.Lights.WPIlib;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import edu.wpi.first.wpilibj.LEDPattern;

public class SetBlinkingPattern extends Command {
    private final LEDSubsystem_WPIlib ledSubsystem;
    private final LEDSubsystem_WPIlib.LEDTarget target;
    private final LEDPattern pattern;
    private final double onTime;
    private final double offTime;

    public SetBlinkingPattern(LEDSubsystem_WPIlib subsystem, LEDSubsystem_WPIlib.LEDTarget target, LEDPattern pattern, double onTime, double offTime) {
        this.ledSubsystem = subsystem;
        this.target = target;
        this.pattern = pattern;
        this.onTime = onTime;
        this.offTime = offTime;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Blinking(target, pattern, onTime, offTime);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
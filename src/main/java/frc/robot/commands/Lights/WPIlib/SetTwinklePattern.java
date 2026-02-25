package frc.robot.commands.Lights.WPIlib;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import edu.wpi.first.wpilibj.util.Color;

public class SetTwinklePattern extends Command {
    private final LEDSubsystem_WPIlib ledSubsystem;
    private final Color baseColor;
    private final Color twinkleColor;
    private final double period;

    public SetTwinklePattern(LEDSubsystem_WPIlib subsystem, Color baseColor, Color twinkleColor, double period) {
        this.ledSubsystem = subsystem;
        this.baseColor = baseColor;
        this.twinkleColor = twinkleColor;
        this.period = period;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_Twinkle(baseColor, twinkleColor, period);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
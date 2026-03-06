package frc.robot;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;

public class MatchInformation extends SubsystemBase{
    LEDSubsystem_WPIlib normalLights;

    // Match Phases
    public enum MatchPhase {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        ENDGAME,
        TRANSITION,
        UNKNOWN
    }

    // Driver Station Info - Required for ALL other calculations
    public boolean enabled;
    public boolean autonomous;
    public boolean teleop;
    public boolean disabled;
    public Optional<DriverStation.Alliance> alliance;
    public DriverStation.MatchType matchType;
    public int matchNumber;
    public double matchTime;
    public String gameData;

    // Phases + Shifts
    public MatchPhase phase;
    public double phaseElapsed;

    public boolean hubActive;
    public int teleopShift;
    public double shiftTimeRemaining;
    public boolean shift1Active;
    public boolean redInactiveFirst;
    public boolean shouldWarn;
    public boolean warning;

    // Flags
    public boolean endgame;
    public boolean climbAdvised;
    public boolean scoringSafeWindow;

    // Time
    public double fpgaTimestamp;
    public double teleopStartTimestamp;
    public double autoStartTimestamp;

    public MatchInformation(LEDSubsystem_WPIlib m_normalLights) {
        this.normalLights = m_normalLights;
        revertDefaultState();
    }

    /** Revert all match info to default state. */
    public void revertDefaultState() {
        enabled = false;
        autonomous = false;
        teleop = false;
        disabled = true;

        alliance = Optional.empty();
        matchType = DriverStation.MatchType.None;
        matchNumber = -1;
        matchTime = -1;
        gameData = "";

        phase = MatchPhase.DISABLED;
        shiftTimeRemaining = 0;
        phaseElapsed = 0;

        hubActive = false;
        teleopShift = 0;
        shift1Active = false;
        redInactiveFirst = false;

        endgame = false;
        climbAdvised = false;
        scoringSafeWindow = false;

        fpgaTimestamp = 0;
        teleopStartTimestamp = -1;
        autoStartTimestamp = -1;
    }

    /**
     * Update [Driver Station] information. ! Run first - other data is based on this. !
     */
    public void updateDriverStation() {
        enabled = DriverStation.isEnabled();
        autonomous = DriverStation.isAutonomous();
        teleop = DriverStation.isTeleop();
        disabled = DriverStation.isDisabled();

        alliance = DriverStation.getAlliance();
        matchType = DriverStation.getMatchType();
        matchNumber = DriverStation.getMatchNumber();
        matchTime = DriverStation.getMatchTime();
        gameData = DriverStation.getGameSpecificMessage();
    }

    /** Update timestamps and calculate elapsed time for phases. ! Run after updating Driver Station info. !
     */
    public void updateTimestamps() {
        fpgaTimestamp = Timer.getFPGATimestamp();

        if (autonomous && autoStartTimestamp < 0)
            autoStartTimestamp = fpgaTimestamp;

        if (teleop && teleopStartTimestamp < 0)
            teleopStartTimestamp = fpgaTimestamp;

        if (disabled) {
            teleopStartTimestamp = -1;
            autoStartTimestamp = -1;
        }
    }

    /** Update the current match phase based on Driver Station info and timestamps. ! Run after updating Driver Station info and timestamps. !
     */
    public void updatePhase() {

        if (disabled) {
            phase = MatchPhase.DISABLED;
            phaseElapsed = 0;
            return;
        }

        if (autonomous) {
            phase = MatchPhase.AUTONOMOUS;
            if (autoStartTimestamp >= 0)
                phaseElapsed = fpgaTimestamp - autoStartTimestamp;
            return;
        }

        if (teleop) {

            if (matchTime > 130) {
                phase = MatchPhase.TRANSITION;
                endgame = false;
                phaseElapsed = 0;
                return;
            }

            if (matchTime >= 0 && matchTime <= 30) {
                phase = MatchPhase.ENDGAME;
                endgame = true;
            } else {
                phase = MatchPhase.TELEOP;
                endgame = false;
            }

            if (teleopStartTimestamp >= 0)
                phaseElapsed = fpgaTimestamp - teleopStartTimestamp;

            return;
        }

        phase = MatchPhase.UNKNOWN;
    }

    /** Update hub and shift logic based on match time and game data. ! Run after updating Driver Station info and timestamps. !
     */
    public void updateHubLogic() {

        teleopShift = 0;
        shiftTimeRemaining = 0;

        // No alliance → cannot compute
        if (alliance.isEmpty()) {
            hubActive = false;
            return;
        }

        // Autonomous → hub always active
        if (autonomous) {
            hubActive = true;
            return;
        }

        // Not teleop → no hub logic
        if (!teleop || matchTime < 0) {
            hubActive = true;
            return;
        }

        // Missing game data → assume active
        if (gameData.isEmpty()) {
            hubActive = true;
            return;
        }

        // Decode auto winner
        char c = gameData.charAt(0);
        if (c != 'R' && c != 'B') {
            hubActive = true;
            return;
        }

        redInactiveFirst = (c == 'R');

        shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        // Shifts
        if (matchTime > 130) {
            hubActive = true;
            return;
        }
        else if (matchTime > 105) {
            teleopShift = 1;
            hubActive = shift1Active;
        }
        else if (matchTime > 80) {
            teleopShift = 2;
            hubActive = !shift1Active;
        }
        else if (matchTime > 55) {
            teleopShift = 3;
            hubActive = shift1Active;
        }
        else if (matchTime > 30) {
            teleopShift = 4;
            hubActive = !shift1Active;
        }
        else {
            hubActive = true;
            return;
        }

        // determining flashing
        if (130 > matchTime && matchTime > 125) {
            shouldWarn = true;
            warning = true;
            return;
        }
        else if (105 > matchTime && matchTime > 100) {
            shouldWarn = true;
            warning = !hubActive;
        }
        else if (80 > matchTime && matchTime > 75) {
            shouldWarn = true;
            warning = !hubActive;
        }
        else if (55 > matchTime && matchTime > 50) {
            shouldWarn = true;
            warning = !hubActive;
        }
        else if (30 > matchTime && matchTime > 25) {
            shouldWarn = true;
            warning = !hubActive;
        }
        else {
            shouldWarn = false;
            warning = false;
            return;
        }
        double cycleElapsed = 130 - matchTime;
        double intoShift = cycleElapsed % 25.0;
        shiftTimeRemaining = 25.0 - intoShift;

        if (shiftTimeRemaining >= 25.0)
            shiftTimeRemaining = 0;
    }

    /** Update convenience flags based on current match state. ! Run after updating Driver Station info, timestamps, and hub logic. !
     * - climbAllowed: true during endgame, false otherwise
     * - scoringSafeWindow: true when hub is active and it's not endgame, false otherwise
     */
    public void updateFlags() {
        climbAdvised = endgame;
        scoringSafeWindow = hubActive && !endgame;
    }

    public boolean isMatchConnected() {
        return enabled && matchTime >= 0;
    }

    /** Periodic update wrapper. */
    public void updateAll() {
        updateDriverStation();
        updateTimestamps();
        updatePhase();
        updateHubLogic();
        updateFlags();
    }

    @Override
    public void periodic() {
        updateAll();
        if (shouldWarn) {
            normalLights.updateBrightness(100);
            if (warning){
                normalLights
                    .LED_Blinking(
                        LEDPattern.solid(LightsConstants.RBGColors.get("green")), 
                        0.5, 
                        0.5);
            } else {
                normalLights
                    .LED_Blinking(
                        LEDPattern.solid(LightsConstants.RBGColors.get("red")), 
                        0.5, 
                        0.5);
            }
        } else {
            if (LEDSubsystem_WPIlib.brightness != LightsConstants.led_brightness){
                normalLights.updateBrightness(50);
            }
        }
    }
}
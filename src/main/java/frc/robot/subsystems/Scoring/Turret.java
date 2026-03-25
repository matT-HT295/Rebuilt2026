// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Scoring;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.Constants.FieldConstants.ScoringZone;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib.LEDTarget;
import frc.util.Interpolation.LoggedTunableNumber;
import frc.robot.Constants.TurretConstants.SystemState;
import edu.wpi.first.wpilibj.Timer;

public class Turret extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    // private final LEDSubsystem_WPIlib leds;
    /* MOTORS */
    private TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorID, "rio");
    private TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();

    /* ENCODERS */
    private CANcoder encoder = new CANcoder(TurretConstants.encoderID, "rio");

    // for position control
    private double position = 0.0;
    private double CCWlimit = 0.85;
    private double CWLimit = -0.85;
    private double gearRatio = 38.8888888889;
    private double offset = 0.00;

    final PositionVoltage mmE_request = new PositionVoltage(0);

    /* PIDFF CONTROL */
    private LoggedTunableNumber k_S = new LoggedTunableNumber("turret_s", TurretConstants.turretSVA[0]);
    private LoggedTunableNumber k_V = new LoggedTunableNumber("turret_v", TurretConstants.turretSVA[1]);
    private LoggedTunableNumber k_A = new LoggedTunableNumber("turret_a", TurretConstants.turretSVA[2]);

    private LoggedTunableNumber k_P = new LoggedTunableNumber("turret_p", TurretConstants.turretPID[0]);
    private LoggedTunableNumber k_I = new LoggedTunableNumber("turret_i", TurretConstants.turretPID[1]);
    private LoggedTunableNumber k_D = new LoggedTunableNumber("turret_d", TurretConstants.turretPID[2]);

    /* STATES */
    TurretWantedState wantedState = TurretWantedState.IDLE;
    SystemState systemState = SystemState.IDLING;

    /** Creates a new Turret */
    public Turret(CommandSwerveDrivetrain drivetrain/* , */ /* LEDSubsystem_WPIlib leds */) {
        this.drivetrain = drivetrain;
        // this.leds = leds;

        /* SETUP CONFIG */

        // CURRENT LIMITS
        turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyCurrentLimit;
        turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorCurrentLimit;

        // turretMotorConfig.Feedback.FeedbackRemoteSensorID = 54;
        turretMotorConfig.Feedback.FeedbackRemoteSensorID = 50;

        // turretMotorConfig.Feedback.FeedbackSensorSource =
        // FeedbackSensorSourceValue.RemoteCANcoder;
        turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turretMotorConfig.Feedback.SensorToMechanismRatio = gearRatio;
        turretMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

        // PID CONSTANTS
        turretMotorConfig.Slot0.kS = k_S.get();
        turretMotorConfig.Slot0.kV = k_V.get();
        turretMotorConfig.Slot0.kA = k_A.get();
        turretMotorConfig.Slot0.kP = k_P.get();
        turretMotorConfig.Slot0.kI = k_I.get();
        turretMotorConfig.Slot0.kD = k_D.get();

        // use this for velocity motion magic
        turretMotorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turretMotionMagicAccel; // Target
                                                                                                        // acceleration
                                                                                                        // of 160 rps/s
                                                                                                        // (0.5 seconds)
        turretMotorConfig.MotionMagic.MotionMagicJerk = TurretConstants.turretMotionMagicJerk;

        // use this for motion magic expo (very good control of position)
        turretMotorConfig.MotionMagic.MotionMagicExpo_kV = TurretConstants.turretMotionMagicExpoK_V;
        turretMotorConfig.MotionMagic.MotionMagicExpo_kA = TurretConstants.turretMotionMagicExpoK_A;

        // APPLY CONFIG TO MOTOR
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = turretMotor.getConfigurator().apply(turretMotorConfig);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
        turretMotor.setPosition(encoder.getAbsolutePosition().getValue());

    }

    public void setWantedTurretState(TurretWantedState desiredState) {
        this.wantedState = desiredState;
    }

    public void applyRightOffset() {
        offset += 0.01;
    }

    public void applyLeftOffset() {
        offset -= 0.01;
    }

    private SystemState changeCurrentSystemState() {
        return switch (wantedState) {
            case IDLE:
                yield SystemState.IDLING;
            case IDLE_AIM:
                yield SystemState.IDLE_AIMING;
            case AIM_HUB:
                yield SystemState.HUB_AIMING;
            case AIM_PASS:
                yield SystemState.PASS_AIMING;
            case TRENCH_PRESETL:
                yield SystemState.TRENCH_PRESETTINGL;
            case TRENCH_PRESETR:
                yield SystemState.TRENCH_PRESETTINGR;
            case HUB_PRESET:
                yield SystemState.HUB_PRESETTING;
            case TEST:
                yield SystemState.TESTING;

        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                position = 0.0;
                break;
            case IDLE_AIMING:

                double target1 = 0;
                double currentTurretToRobotAngle1 = turretMotor.getPosition().getValueAsDouble();
                // calculate robot angle relative to field
                Rotation2d currentRobotAngle1 = drivetrain.getTurretPose().getRotation();
                Rotation2d angleToHub1 = drivetrain.getSOTFTurretAngle().getAngle();

                // calculate desired angle of turret relative to hub
                // double angleToHub = (Math.atan2(passSpot.getY(), passSpot.getX()));

                // calculate desired angle of turret relative to robot
                Rotation2d desiredTurretAngle1 = (angleToHub1).minus(currentRobotAngle1);
                // convert to rotations
                double convertedTurretAngle1 = desiredTurretAngle1.getDegrees() / 360;

                // compute shortest delta between branches
                double delta1 = convertedTurretAngle1 - (currentTurretToRobotAngle1);
                delta1 = Math.IEEEremainder(delta1, 1.0);

                // now apply
                target1 = currentTurretToRobotAngle1 + delta1;

                // now enforce mechanical limits with wrap only if truly needed
                while (target1 > CCWlimit)
                    target1 -= 1.0;
                while (target1 < CWLimit)
                    target1 += 1.0;

                position = target1;
                break;
            case PASS_AIMING:
                double target = 0;
                // leds.LED_ScrollPatternRelative(LEDPattern.gradient(GradientType.kContinuous,
                // Color.kOrange, Color.kYellow), 2.5);
     double currentTurretToRobotAngle = turretMotor.getPosition().getValueAsDouble();
    Rotation2d currentRobotAngle = drivetrain.getPose().getRotation();

    // Correct for staleness of shot command
    double dt = Timer.getFPGATimestamp() - drivetrain.shotCommandTimestamp;
    double omegaRad = drivetrain.getState().Speeds.omegaRadiansPerSecond;
    Rotation2d rotationCorrection = Rotation2d.fromRadians(omegaRad * dt);

    Rotation2d angleToHub = currentShotCommand.turretAngle()
    .plus(rotationCorrection);
    
    Rotation2d desiredTurretAngle = (angleToHub)
            .minus(currentRobotAngle)
            .plus(Rotation2d.fromDegrees(180));
    double convertedTurretAngle = desiredTurretAngle.getDegrees() / 360;
    double delta = convertedTurretAngle - (currentTurretToRobotAngle);
    delta = Math.IEEEremainder(delta, 1.0);
    target = currentTurretToRobotAngle + delta;
    while (target > CCWlimit)
        target -= 1.0;
    while (target < CWLimit)
        target += 1.0;
    target += offset;
    position = target;
    break;
           // case HUB_AIMING:
                // double target2 = 0;
                // // leds.LED_ScrollPatternRelative(LEDPattern.gradient(GradientType.kContinuous,
                // // Color.kCadetBlue, Color.kLightGreen), 2.5);
                // double currentTurretToRobotAngle2 = turretMotor.getPosition().getValueAsDouble();
                // // calculate robot angle relative to field
                // Rotation2d currentRobotAngle2 = drivetrain.getPose().getRotation();
                // Rotation2d angleToHub2 = drivetrain.getSOTFTurretAngle().getAngle();

                // /* OPTION 2 */
                // // angleToHub = drivetrain.SOTF_CALC().getAngle()

                // /* OPTION 3 */
                // // ChassisSpeeds rawFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                // // drivetrain.getState().Speeds,
                // // drivetrain.getPose().getRotation());
                // // Rotation2d angleToHub2 = Rotation2d.fromDegrees(ShotCalc.calculateSOTF(
                // // drivetrain.getTurretPose().getTranslation(),
                // // rawFieldSpeeds, drivetrain.getScoringLocation(),
                // // ShooterConstants.latencyCompensation).turretAngle());

                // // calculate desired angle of turret relative to hub
                // // double angleToHub2 = (Math.atan2(drivetrain.getYfromHub(),
                // // drivetrain.getXfromHub()));

                // // calculate desired angle of turret relative to robot
                // Rotation2d desiredTurretAngle2 = (angleToHub2)
                //         .minus(currentRobotAngle2)
                //         .plus(Rotation2d.fromDegrees(180));
                // // Rotation2d desiredTurretAngle2 = currentRobotAngle2;
                // // convert to rotations
                // double convertedTurretAngle2 = desiredTurretAngle2.getDegrees() / 360;

                // // compute shortest delta between branches
                // double delta2 = convertedTurretAngle2 - (currentTurretToRobotAngle2);
                // delta2 = Math.IEEEremainder(delta2, 1.0);

                // // now apply
                // target2 = currentTurretToRobotAngle2 + delta2;

                // // now enforce mechanical limits with wrap only if truly needed
                // while (target2 > CCWlimit)
                //     target2 -= 1.0;
                // while (target2 < CWLimit)
                //     target2 += 1.0;

                // // probe
                // target2 += offset;
                // // SmartDashboard.putNumber("Turret Setpoint with adjustment", target2);

                // position = target2;

                //-----------------------------------------

                case HUB_AIMING:
    double target2 = 0;
    double currentTurretToRobotAngle2 = turretMotor.getPosition().getValueAsDouble();
    Rotation2d currentRobotAngle2 = drivetrain.getPose().getRotation();

    // Correct for staleness of shot command
    double dt2 = Timer.getFPGATimestamp() - drivetrain.shotCommandTimestamp;
    double omegaRad2 = drivetrain.getState().Speeds.omegaRadiansPerSecond;
    Rotation2d rotationCorrection2 = Rotation2d.fromRadians(omegaRad2 * dt2);

    Rotation2d angleToHub2 = drivetrain.currentShotCommand.turretAngle()
    .plus(rotationCorrection2);
    
    Rotation2d desiredTurretAngle2 = (angleToHub2)
            .minus(currentRobotAngle2)
            .plus(Rotation2d.fromDegrees(180));
    double convertedTurretAngle2 = desiredTurretAngle2.getDegrees() / 360;
    double delta2 = convertedTurretAngle2 - (currentTurretToRobotAngle2);
    delta2 = Math.IEEEremainder(delta2, 1.0);
    target2 = currentTurretToRobotAngle2 + delta2;
    while (target2 > CCWlimit)
        target2 -= 1.0;
    while (target2 < CWLimit)
        target2 += 1.0;
    target2 += offset;
    position = target2;
    break;
    
            case TRENCH_PRESETTINGL:
                position = TurretConstants.trenchPresetPositionL;
                break;
            case TRENCH_PRESETTINGR:
                position = TurretConstants.trenchPresetPositionR;
                break;
            case HUB_PRESETTING:
                position = TurretConstants.hubPresetPosition;
                break;
            case TESTING:
                position = .75;
                break;
        }
    }

    public boolean turretIsReady() {
        if ((turretMotor.getPosition().getValueAsDouble() - position) < TurretConstants.tolerance) {
            return true;
        } else {
            return false;
        }
    }

    public void enableEcoModeTurret() {
        turretMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        turretMotor.getConfigurator().apply(turretMotorConfig);
    }

    public void disableEcoModeTurret() {
        turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorCurrentLimit;
        turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyCurrentLimit;
        turretMotor.getConfigurator().apply(turretMotorConfig);
    }

    /**
     * Check LoggedTunableNumbers. If changed, update PID and SVA values of motor
     */
    public void checkTunableValues() {
        if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged()
                || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
            turretMotorConfig.Slot0.kS = k_S.get();
            turretMotorConfig.Slot0.kV = k_V.get();
            turretMotorConfig.Slot0.kA = k_A.get();
            turretMotorConfig.Slot0.kP = k_P.get();
            turretMotorConfig.Slot0.kI = k_I.get();
            turretMotorConfig.Slot0.kD = k_D.get();

            turretMotor.getConfigurator().apply(turretMotorConfig);

        }
    }

    public TurretWantedState getState() {
        return wantedState;
    }

    private void logValues() {
        SmartDashboard.putNumber("Turret Absolute Position", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret Motor Position", turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret Wanted Position", position);
        SmartDashboard.putNumber("Turret Encoder Position", encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret Offset", offset);
        SmartDashboard.putString("TURRET WANTED STATE", wantedState.toString());
        SmartDashboard.putString("TURRET SYSTEM STATE", systemState.toString());
    }

    @Override
    public void periodic() {
        logValues();
        systemState = changeCurrentSystemState();
        applyState();
        // example of how to control motor for position
        turretMotor.setControl(mmE_request.withPosition(position));
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Scoring;

import edu.wpi.first.math.util.Units;

import org.opencv.core.Mat;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.Constants.TurretConstants.SystemState;
import frc.util.LoggedTunableNumber;

public class Turret extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final LEDSubsystem_WPIlib leds;
  /* MOTORS */
  private TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorID, "rio");
  private TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
  
  /* ENCODERS */
  private CANcoder encoder = new CANcoder(TurretConstants.encoderID, "rio");

  //for position control
  private double position = 0.0;
  private double CCWlimit = 0.85;
  private double CWLimit = -0.85;
  private double gearRatio = 38.8888888889;

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
  public Turret(CommandSwerveDrivetrain drivetrain, LEDSubsystem_WPIlib leds) {
    this.drivetrain = drivetrain;
    this.leds = leds;

    /* SETUP CONFIG */
    
    // CURRENT LIMITS
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyCurrentLimit;
    turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorCurrentLimit;
    

    // turretMotorConfig.Feedback.FeedbackRemoteSensorID = 54;
    turretMotorConfig.Feedback.FeedbackRemoteSensorID = 50;
    
    // turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    turretMotorConfig.Feedback.SensorToMechanismRatio = gearRatio;
    turretMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    
    //PID CONSTANTS
    turretMotorConfig.Slot0.kS = k_S.get();
    turretMotorConfig.Slot0.kV = k_V.get();
    turretMotorConfig.Slot0.kA = k_A.get(); 
    turretMotorConfig.Slot0.kP = k_P.get();
    turretMotorConfig.Slot0.kI = k_I.get();
    turretMotorConfig.Slot0.kD = k_D.get();

    //use this for velocity motion magic 
    turretMotorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turretMotionMagicAccel; // Target acceleration of 160 rps/s (0.5 seconds)
    turretMotorConfig.MotionMagic.MotionMagicJerk = TurretConstants.turretMotionMagicJerk;

    //use this for motion magic expo (very good control of position)
    turretMotorConfig.MotionMagic.MotionMagicExpo_kV = TurretConstants.turretMotionMagicExpoK_V;
    turretMotorConfig.MotionMagic.MotionMagicExpo_kA = TurretConstants.turretMotionMagicExpoK_A;

    //APPLY CONFIG TO MOTOR
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = turretMotor.getConfigurator().apply(turretMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    turretMotor.setPosition(encoder.getAbsolutePosition().getValue());

  }

  public void setWantedTurretState(TurretWantedState desiredState) {
    this.wantedState = desiredState;
  }

  private SystemState changeCurrentSystemState() {
    return switch (wantedState) {
      case IDLE: 
        yield SystemState.IDLING;
      case AIM_HUB:
          yield SystemState.HUB_AIMING;
      case AIM_PASS:
        yield SystemState.PASS_AIMING;
      case TRENCH_PRESET:
        yield SystemState.TRENCH_PRESETTING;
      case HUB_PRESET:
        yield SystemState.HUB_PRESETTING;
      case TEST:
        yield SystemState.TESTING;
        
    };
  }
  

    
  private void applyState(){
    switch(systemState){
      case IDLING:
        position = 0.0;
        break;
      case PASS_AIMING:
        double target = 0;
        Translation2d passSpot;
        if(DriverStation.getAlliance().get() == Alliance.Red) {
          if(drivetrain.getPose().getY() > 4.03) {
            passSpot = new Translation2d(15.5, 7);
          } else {
            passSpot = new Translation2d(15.5, 1);
          }
        } else {
          if(drivetrain.getPose().getY() > 4.03) {
            passSpot = new Translation2d(1, 7);
          } else {
            passSpot = new Translation2d(1, 1);
          }
        }
        // leds.LED_ScrollPatternRelative(LEDPattern.gradient(GradientType.kContinuous, Color.kOrange, Color.kYellow), 2.5);
        double currentTurretToRobotAngle = turretMotor.getPosition().getValueAsDouble();
        //calculate robot angle relative to field
        // Rotation2d currentRobotAngle = drivetrain.getTurretPose().getRotation();
        Rotation2d currentRobotAngle = drivetrain.getSOTFTurretAngle().getAngle();

        // calculate desired angle of turret relative to hub
        double angleToHub = (Math.atan2(passSpot.getY(), passSpot.getX()));

        // calculate desired angle of turret relative to robot
        Rotation2d desiredTurretAngle = Rotation2d.fromRadians(angleToHub).minus(currentRobotAngle);
        // convert to rotations
        double convertedTurretAngle = desiredTurretAngle.getDegrees()/360;
        
        // compute shortest delta between branches
        double delta = convertedTurretAngle - (currentTurretToRobotAngle);
        delta = Math.IEEEremainder(delta, 1.0);

        // now apply
        target = currentTurretToRobotAngle + delta;

        // now enforce mechanical limits with wrap only if truly needed
        while (target > CCWlimit) target -= 1.0;
        while (target < CWLimit) target += 1.0;

        // probe
        SmartDashboard.putNumber("Turret Setpoint with adjustment", target);

        position = target;
        break;
      case HUB_AIMING:
        double target2 = 0;
        // leds.LED_ScrollPatternRelative(LEDPattern.gradient(GradientType.kContinuous, Color.kCadetBlue, Color.kLightGreen), 2.5);
        double currentTurretToRobotAngle2 = turretMotor.getPosition().getValueAsDouble();
        //calculate robot angle relative to field
        // Rotation2d currentRobotAngle = drivetrain.getTurretPose().getRotation();
        Rotation2d currentRobotAngle2 = drivetrain.getSOTFTurretAngle().getAngle();

        // calculate desired angle of turret relative to hub
        double angleToHub2 = (Math.atan2(drivetrain.getYfromHub(), drivetrain.getXfromHub()));

        // calculate desired angle of turret relative to robot
        Rotation2d desiredTurretAngle2 = Rotation2d.fromRadians(angleToHub2).minus(currentRobotAngle2);
        // convert to rotations
        double convertedTurretAngle2 = desiredTurretAngle2.getDegrees()/360;
        
        // compute shortest delta between branches
        double delta2 = convertedTurretAngle2 - (currentTurretToRobotAngle2);
        delta2 = Math.IEEEremainder(delta2, 1.0);

        // now apply
        target2 = currentTurretToRobotAngle2 + delta2;

        // now enforce mechanical limits with wrap only if truly needed
        while (target2 > CCWlimit) target2 -= 1.0;
        while (target2 < CWLimit) target2 += 1.0;

        // probe
        SmartDashboard.putNumber("Turret Setpoint with adjustment", target2);

        position = target2;
        break;
      case TRENCH_PRESETTING:
        position = TurretConstants.trenchPresetPosition;
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
    SmartDashboard.putString("TURRET WANTED STATE", wantedState.toString());
    SmartDashboard.putString("TURRET SYSTEM STATE", systemState.toString());
  }

  @Override
  public void periodic() {
    logValues();
    systemState = changeCurrentSystemState();
    applyState();
    //example of how to control motor for position
    turretMotor.setControl(mmE_request.withPosition(position));
  }

}

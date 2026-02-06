// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Scoring;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.Constants.TurretConstants.SystemState;
import frc.util.LoggedTunableNumber;

public class Turret extends SubsystemBase {
  /* MOTORS */
  private TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorID, "rio");
  private TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
  //for position control
  private double position = 0.0;
  final MotionMagicExpoVoltage mmE_request = new MotionMagicExpoVoltage(0);

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
  public Turret() {
    /* SETUP CONFIG */
    
    // CURRENT LIMITS
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyCurrentLimit;
    turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorCurrentLimit;
    
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
  }

  public void setWantedTurretMode(TurretWantedState desiredState) {
    this.wantedState = desiredState;
  }

  private SystemState changeCurrentSystemState() {
    return switch (wantedState) {
      case IDLE: 
        yield SystemState.IDLING;
      case AIM:
        boolean isInAllianceZone = true;
        if (isInAllianceZone) {
          yield SystemState.PASS_AIMING;
        } else {
          yield SystemState.HUB_AIMING;
        }
      case TRENCH_PRESET:
        yield SystemState.TRENCH_PRESETTING;
      case CLOSE_PRESET:
        yield SystemState.CLOSE_PRESETTING;
        
    };
  }

    
  private void applyState(){
    switch(systemState){
      case IDLING:
        position = 0.0;
        break;
      case PASS_AIMING:
        position = TurretConstants.passAimPosition;
        break;
      case HUB_AIMING:
        position = TurretConstants.hubAimPosition;
        break;
      case TRENCH_PRESETTING:
        position = TurretConstants.trenchPresetPosition;
        break;
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
    }
    turretMotor.getConfigurator().apply(turretMotorConfig);
  }

  @Override
  public void periodic() {
    checkTunableValues();
    systemState = changeCurrentSystemState();
    applyState();
    //example of how to control motor for position
    turretMotor.setControl(mmE_request.withPosition(position));
  }

}

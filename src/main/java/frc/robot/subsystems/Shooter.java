// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterWantedState;
import frc.robot.Constants.ShooterConstants.SystemState;
import frc.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  /* MOTORS */
  private TalonFX hoodMotor = new TalonFX(ShooterConstants.hoodMotorID, "rio");
  private TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
  private TalonFX shooterMotor1 = new TalonFX(ShooterConstants.shooterMotor1ID, "rio");
  private TalonFX shooterMotor2 = new TalonFX(ShooterConstants.shooterMotor2ID, "rio");
  private TalonFXConfiguration shooterMotorsConfig = new TalonFXConfiguration();

  //for velocity control
  private double motorspeed = 0.0;
  final MotionMagicVelocityVoltage mm_request = new MotionMagicVelocityVoltage(0);
  //for position control
  private double position = 0.0;
  final MotionMagicExpoVoltage mmE_request = new MotionMagicExpoVoltage(0);

  /* PIDFF CONTROL */
  private LoggedTunableNumber k_S = new LoggedTunableNumber("shooter_s", ShooterConstants.shooterSVA[0]);
  private LoggedTunableNumber k_V = new LoggedTunableNumber("shooter_v", ShooterConstants.shooterSVA[1]);
  private LoggedTunableNumber k_A = new LoggedTunableNumber("shooter_a", ShooterConstants.shooterSVA[2]);
  private LoggedTunableNumber k_P = new LoggedTunableNumber("shooter_p", ShooterConstants.shooterPID[0]);
  private LoggedTunableNumber k_I = new LoggedTunableNumber("shooter_i", ShooterConstants.shooterPID[1]);
  private LoggedTunableNumber k_D = new LoggedTunableNumber("shooter_d", ShooterConstants.shooterPID[2]);

  /* STATES */
  ShooterWantedState wantedState = ShooterWantedState.IDLE;
  SystemState systemState = SystemState.IDLING;

  /** Creates a new Shooter */
  public Shooter() {
    /* SETUP CONFIG */
    
    // CURRENT LIMITS
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
    hoodMotorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
    
    //PID CONSTANTS
    hoodMotorConfig.Slot0.kS = k_S.get();
    hoodMotorConfig.Slot0.kV = k_V.get();
    hoodMotorConfig.Slot0.kA = k_A.get(); 
    hoodMotorConfig.Slot0.kP = k_P.get();
    hoodMotorConfig.Slot0.kI = k_I.get();
    hoodMotorConfig.Slot0.kD = k_D.get();

    shooterMotorsConfig.Slot0.kS = k_S.get();
    shooterMotorsConfig.Slot0.kV = k_V.get();
    shooterMotorsConfig.Slot0.kA = k_A.get(); 
    shooterMotorsConfig.Slot0.kP = k_P.get();
    shooterMotorsConfig.Slot0.kI = k_I.get();
    shooterMotorsConfig.Slot0.kD = k_D.get();

    //use this for velocity motion magic 
    hoodMotorConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.shooterMotionMagicAccel; // Target acceleration of 160 rps/s (0.5 seconds)
    hoodMotorConfig.MotionMagic.MotionMagicJerk = ShooterConstants.shooterMotionMagicJerk;

    shooterMotorsConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.shooterMotionMagicAccel; // Target acceleration of 160 rps/s (0.5 seconds)
    shooterMotorsConfig.MotionMagic.MotionMagicJerk = ShooterConstants.shooterMotionMagicJerk;

    //use this for motion magic expo (very good control of position)
    hoodMotorConfig.MotionMagic.MotionMagicExpo_kV = ShooterConstants.shooterMotionMagicExpoK_V;
    hoodMotorConfig.MotionMagic.MotionMagicExpo_kA = ShooterConstants.shooterMotionMagicExpoK_A;

    shooterMotorsConfig.MotionMagic.MotionMagicExpo_kV = ShooterConstants.shooterMotionMagicExpoK_V;
    shooterMotorsConfig.MotionMagic.MotionMagicExpo_kA = ShooterConstants.shooterMotionMagicExpoK_A;

    //APPLY CONFIG TO MOTOR
    StatusCode hoodMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      hoodMotorStatus = hoodMotor.getConfigurator().apply(hoodMotorConfig);
      if (hoodMotorStatus.isOK()) break;
    }
    if (!hoodMotorStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + hoodMotorStatus.toString());
    }

    StatusCode shooterMotor1Status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      shooterMotor1Status = shooterMotor1.getConfigurator().apply(shooterMotorsConfig);
      if (shooterMotor1Status.isOK()) break;
    }
    if (!shooterMotor1Status.isOK()) {
      System.out.println("Could not apply configs, error code: " + shooterMotor1Status.toString());
    }

    StatusCode shooterMotor2Status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      shooterMotor2Status = shooterMotor2.getConfigurator().apply(shooterMotorsConfig);
      if (shooterMotor2Status.isOK()) break;
    }
    if (!shooterMotor2Status.isOK()) {
      System.out.println("Could not apply configs, error code: " + shooterMotor2Status.toString());
    }
  }

  public void setWantedShooterMode(ShooterWantedState desiredState) {
    this.wantedState = desiredState;
  }

  private SystemState changeCurrentSystemState() {
    /// DriverStation.getGameSpecificMessage();     DriverStation.getAlliance();
    return switch (wantedState) {
      case IDLE:
        yield SystemState.IDLING;
      case WAIT:
        if(DriverStation.getGameSpecificMessage() == getAlliance()){
          if((DriverStation.getMatchTime() <= 105 && DriverStation.getMatchTime() > 80) || (DriverStation.getMatchTime() <= 55 && DriverStation.getMatchTime() > 30)){
            yield SystemState.ACTIVE_WAITING;
          }
          if((DriverStation.getMatchTime() <= 130 && DriverStation.getMatchTime() > 105) || (DriverStation.getMatchTime() <= 80 && DriverStation.getMatchTime() > 55)){
            yield SystemState.INACTIVE_WAITING;
          }
        }
        else{
          if((DriverStation.getMatchTime() <= 130 && DriverStation.getMatchTime() > 105) || (DriverStation.getMatchTime() <= 80 && DriverStation.getMatchTime() > 55)){
            yield SystemState.ACTIVE_WAITING;
          }
          if((DriverStation.getMatchTime() <= 105 && DriverStation.getMatchTime() > 80) || (DriverStation.getMatchTime() <= 55 && DriverStation.getMatchTime() > 30)){
            yield SystemState.INACTIVE_WAITING;
          }
        }
      case PASS_SHOOT:
        yield SystemState.PASS_SHOOTING;
      case HUB_SHOOT:
        yield SystemState.HUB_SHOOTING;
    };
  }

    
  private void applyState(){
    switch(systemState){
      case IDLING:
        motorspeed = 0.0;
        position = 0.0;
        break;
      case ACTIVE_WAITING:
        motorspeed = ShooterConstants.activeWaitingSpeed;
        position = 0.0;
        break;
      case INACTIVE_WAITING:
        motorspeed = ShooterConstants.inactiveWaitingSpeed;
        position = 0.0;
        break;
      case HUB_SHOOTING:
        motorspeed = ShooterConstants.shooterSpeedInterpolation.getPrediction(ShooterConstants.distanceToHub);
        position = ShooterConstants.hoodAngleInterpolation.getPrediction(ShooterConstants.distanceToHub);
        break;
      case PASS_SHOOTING:
        motorspeed = ShooterConstants.shooterSpeedInterpolation.getPrediction(ShooterConstants.passDistance);
        position = ShooterConstants.hoodAngleInterpolation.getPrediction(ShooterConstants.passDistance);
        break;
    }
  }


  /**
   * Check LoggedTunableNumbers. If changed, update PID and SVA values of motor
   */
  public void checkTunableValues() {
    if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged() 
    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
      hoodMotorConfig.Slot0.kS = k_S.get();
      hoodMotorConfig.Slot0.kV = k_V.get();
      hoodMotorConfig.Slot0.kA = k_A.get(); 
      hoodMotorConfig.Slot0.kP = k_P.get();
      hoodMotorConfig.Slot0.kI = k_I.get();
      hoodMotorConfig.Slot0.kD = k_D.get();
    }
    hoodMotor.getConfigurator().apply(hoodMotorConfig);

    if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged() 
    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
      shooterMotorsConfig.Slot0.kS = k_S.get();
      shooterMotorsConfig.Slot0.kV = k_V.get();
      shooterMotorsConfig.Slot0.kA = k_A.get(); 
      shooterMotorsConfig.Slot0.kP = k_P.get();
      shooterMotorsConfig.Slot0.kI = k_I.get();
      shooterMotorsConfig.Slot0.kD = k_D.get();
    }
    shooterMotor1.getConfigurator().apply(shooterMotorsConfig);
    shooterMotor2.getConfigurator().apply(shooterMotorsConfig);
  }

  private String getAlliance(){
    if(DriverStation.getAlliance().toString() == "Alliance.Red"){
      return "R";
    } else{
      return "B";
    }
  }

  @Override
  public void periodic() {
    checkTunableValues();
    systemState = changeCurrentSystemState();
    applyState();
    //example of how to control motor for position
    hoodMotor.setControl(mmE_request.withPosition(position));
    //example of how to control motor for velocity
    shooterMotor1.setControl(mm_request.withVelocity(motorspeed));
    //example of how to control motor for velocity
    shooterMotor2.setControl(mm_request.withVelocity(-motorspeed));
  }

}
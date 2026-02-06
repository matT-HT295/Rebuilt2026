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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FeederConstants.FeederWantedState;
import frc.robot.Constants.FeederConstants.SystemState;
import frc.util.LoggedTunableNumber;

public class Feeder extends SubsystemBase {
  /* MOTORS */
  private TalonFX intakeFeederMotor = new TalonFX(FeederConstants.intakeFeederMotorID, "rio");
  private TalonFXConfiguration intakeFeederMotorConfig = new TalonFXConfiguration();
  private TalonFX shootFeederMotor = new TalonFX(FeederConstants.shootFeederMotorID, "rio");
  private TalonFXConfiguration shootFeederMotorConfig = new TalonFXConfiguration();
  
  //for velocity control
  private double intakemotorspeed = 0.0;
  private double shootmotorspeed = 0.0;
  final MotionMagicVelocityVoltage mm_request = new MotionMagicVelocityVoltage(0);
  //for position control
  private double position = 0.0;
  final MotionMagicExpoVoltage mmE_request = new MotionMagicExpoVoltage(0);

  /* PIDFF CONTROL */
  private LoggedTunableNumber k_S = new LoggedTunableNumber("feeder_s", FeederConstants.feederSVA[0]);
  private LoggedTunableNumber k_V = new LoggedTunableNumber("feeder_v", FeederConstants.feederSVA[1]);
  private LoggedTunableNumber k_A = new LoggedTunableNumber("feeder_a", FeederConstants.feederSVA[2]);
  private LoggedTunableNumber k_P = new LoggedTunableNumber("feeder_p", FeederConstants.feederPID[0]);
  private LoggedTunableNumber k_I = new LoggedTunableNumber("feeder_i", FeederConstants.feederPID[1]);
  private LoggedTunableNumber k_D = new LoggedTunableNumber("feeder_d", FeederConstants.feederPID[2]);

  /* STATES */
  FeederWantedState wantedState = FeederWantedState.IDLE;
  SystemState systemState = SystemState.IDLING;

  /** Creates a new Feeder */
  public Feeder() {
    /* SETUP CONFIG */
    
    // CURRENT LIMITS
    intakeFeederMotorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.SupplyCurrentLimit;
    intakeFeederMotorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.StatorCurrentLimit;
    shootFeederMotorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.SupplyCurrentLimit;
    shootFeederMotorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.StatorCurrentLimit;

    //APPLY CONFIG TO MOTOR
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeFeederMotor.getConfigurator().apply(intakeFeederMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  
    for (int i = 0; i < 5; ++i) {
      status = shootFeederMotor.getConfigurator().apply(shootFeederMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void setWantedFeederMode(FeederWantedState desiredState) {
    this.wantedState = desiredState;
  }

  private SystemState changeCurrentSystemState() {
    return switch (wantedState) {
      case IDLE: 
        yield SystemState.IDLING;
      case INTAKE: 
        yield SystemState.INTAKING;
      case SHOOT: 
        yield SystemState.SHOOTING;
    };
  }

    
  private void applyState(){
    switch(systemState){
      case IDLING:
        intakemotorspeed = 0.0;
        shootmotorspeed = 0.0;
        break;
      case INTAKING:
        intakemotorspeed = FeederConstants.feederIntakeSpeed;
        shootmotorspeed = 0.0;
        break;
      case SHOOTING:
        shootmotorspeed = FeederConstants.feederShootSpeed;
        intakemotorspeed = FeederConstants.feederIntakeSpeed;
        break;
    }
  }


  /**
   * Check LoggedTunableNumbers. If changed, update PID and SVA values of motor
   */


  @Override
  public void periodic() {
    systemState = changeCurrentSystemState();
    applyState();
    //example of how to control motor for velocity
    shootFeederMotor.setControl(mm_request.withVelocity(shootmotorspeed));
    //example of how to control motor for position
    shootFeederMotor.setControl(mmE_request.withPosition(position));
    intakeFeederMotor.setControl(mm_request.withVelocity(intakemotorspeed));
    //example of how to control motor for position
    intakeFeederMotor.setControl(mmE_request.withPosition(position));
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeWantedState;
import frc.robot.Constants.IntakeConstants.SystemState;
import frc.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  /* MOTORS */
  private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, "rio");
  private TalonFX intakeExtensionMotor = new TalonFX(IntakeConstants.intakeExtensionMotorID, "rio");
  private TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
  //for velocity control
  private double motorspeed = 0.0;
  final MotionMagicVelocityVoltage mm_request = new MotionMagicVelocityVoltage(0);
  //for position control
  private double position = 0.0;
  final MotionMagicExpoVoltage mmE_request = new MotionMagicExpoVoltage(0);
  final DutyCycleOut m_leftrequestOut = new DutyCycleOut(0.0);
  
  private LoggedTunableNumber k_S = new LoggedTunableNumber("intake_s", IntakeConstants.intakeSVA[0]);
  private LoggedTunableNumber k_V = new LoggedTunableNumber("intake_v", IntakeConstants.intakeSVA[1]);
  private LoggedTunableNumber k_A = new LoggedTunableNumber("intake_a", IntakeConstants.intakeSVA[2]);
  private LoggedTunableNumber k_P = new LoggedTunableNumber("intake_p", IntakeConstants.intakePID[0]);
  private LoggedTunableNumber k_I = new LoggedTunableNumber("intake_i", IntakeConstants.intakePID[1]);
  private LoggedTunableNumber k_D = new LoggedTunableNumber("intake_d", IntakeConstants.intakePID[2]);
  /* STATES */
  IntakeWantedState wantedState = IntakeWantedState.IDLE;
  SystemState systemState = SystemState.IDLING;
  

  /** Creates a new Intake */
  public Intake() {
    /* SETUP CONFIG */
    intakeMotorConfig.Slot0.kS = k_S.get();
    intakeMotorConfig.Slot0.kV = k_V.get();
    intakeMotorConfig.Slot0.kA = k_A.get(); 
    intakeMotorConfig.Slot0.kP = k_P.get();
    intakeMotorConfig.Slot0.kI = k_I.get();
    intakeMotorConfig.Slot0.kD = k_D.get();

    // CURRENT LIMITS
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SupplyCurrentLimit;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.StatorCurrentLimit;

    //Motion MAgic constants
    intakeMotorConfig.MotionMagic.MotionMagicExpo_kA = IntakeConstants.intakeMotionMagicExpoK_A;    
    intakeMotorConfig.MotionMagic.MotionMagicExpo_kV = IntakeConstants.intakeMotionMagicExpoK_V;
    intakeMotorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.intakeMotionMagicCruiseVel;

    //APPLY CONFIG TO MOTOR
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor.getConfigurator().apply(intakeMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /**
   * Check LoggedTunableNumbers. If changed, update PID and SVA values of motor
   */
  public void checkTunableValues() {
    if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged() 
    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
      intakeMotorConfig.Slot0.kS = k_S.get();
      intakeMotorConfig.Slot0.kV = k_V.get();
      intakeMotorConfig.Slot0.kA = k_A.get(); 
      intakeMotorConfig.Slot0.kP = k_P.get();
      intakeMotorConfig.Slot0.kI = k_I.get();
      intakeMotorConfig.Slot0.kD = k_D.get();
      intakeExtensionMotor.getConfigurator().apply(intakeMotorConfig);
    }
  }

  public void setWantedIntakeState(IntakeWantedState desiredState) {
    this.wantedState = desiredState;
  }

  private SystemState changeCurrentSystemState() {
    return switch (wantedState) {
      case IDLE: 
        yield SystemState.IDLING;
      case SHOOT :
        yield SystemState.SHOOTING;
      case INTAKE :
        yield SystemState.INTAKING;
      case RETRACT :
        yield SystemState.RETRACTING;
      case RESET:
        yield SystemState.RESETING;
    };
  }

    
  private void applyState(){
    switch(systemState){
      case IDLING:
        motorspeed = 0.0;
        break;
      case INTAKING:
        position = IntakeConstants.intakingPosition; 
        motorspeed = IntakeConstants.intakingSpeed;
        break;
      case SHOOTING :
        motorspeed = IntakeConstants.shootingPosition;
        break;
      case RETRACTING :
        position = IntakeConstants.retractingPos;
        break;
      case RESETING:
        intakeExtensionMotor.setPosition(0);
        break;
    }
  }
  
  private void LogValues() {
    SmartDashboard.putNumber("Extension Motor Position", intakeExtensionMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake actual speed", intakeMotor.get());
    SmartDashboard.putNumber("Extension Wanted Position", position);
    SmartDashboard.putNumber("Intake Wanted Speed", motorspeed);
    SmartDashboard.putString("INTAKE WANTED STATE", wantedState.toString());
    SmartDashboard.putString("INTAKE SYSTEM STATE", systemState.toString());
  }

  @Override
  public void periodic() {
    LogValues();
    checkTunableValues();
    systemState = changeCurrentSystemState();
    applyState();
    //example of how to control motor for velocity
    // intakeMotor.setControl(mm_request.withVelocity(motorspeed));
    //example of how to control motor for position
    intakeExtensionMotor.setControl(mmE_request.withPosition(position));
    // setting the request to the motor controller
    intakeMotor.setControl(m_leftrequestOut.withOutput(motorspeed));
  }

}
 
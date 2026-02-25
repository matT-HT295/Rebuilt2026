// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.FeederConstants.FeederWantedState;
import frc.robot.Constants.IntakeConstants.IntakeWantedState;
import frc.robot.Constants.ShooterConstants.ShooterWantedState;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Scoring.Shooter;
import frc.robot.subsystems.Scoring.Turret;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Intake.Feeder;
import frc.robot.subsystems.Intake.Intake;

import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.commands.Lights.WPIlib.RunPattern;
import frc.robot.commands.Lights.WPIlib.ScrollPattern;
import frc.robot.commands.Lights.WPIlib.SetBreathingPattern;
import frc.robot.commands.Lights.WPIlib.SetSolidColor;
import frc.robot.commands.Lights.WPIlib.SetTwinklePattern;
import frc.robot.commands.Lights.WPIlib.ResetLED;
import frc.robot.commands.Lights.WPIlib.DisableLED;
import frc.robot.commands.Lights.WPIlib.ReselLED_OLD;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LEDSubsystem_WPIlib normalLights = new LEDSubsystem_WPIlib();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Intake intake = new Intake();
  private final Feeder feeder = new Feeder();
  private final Turret turret = new Turret(drivetrain);
  private final Shooter shooter = new Shooter();
  // private final Vision vision = new Vision();
  public SendableChooser<Command> sendableChooser = new SendableChooser<>();


  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorControllerPort);


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
        
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutoCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.registerTelemetry(logger::telemeterize);
    
    /* DRIVER CONTROLS */
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );
    //gyro reset
    driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    
    //intake
    driver.rightBumper()
      .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.INTAKE)))
      .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.IDLE)));
    driver.leftBumper()
      .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.RETRACT)))
      .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(IntakeWantedState.IDLE)));

    // Driver Helping
    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driver.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
    // ));  

    /* OPERATOR */



    /* TESTING BUTTONS */
    //shooting
    operator.rightTrigger()
      .onTrue(new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TEST)),
        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.FEEDTEST))))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.IDLE)),
        new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE))));

    //feeder
    driver.y()
      .onTrue(new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.FEEDTEST)))
      .onFalse(new InstantCommand(() -> feeder.setWantedFeederState(FeederWantedState.IDLE)));
    
    //lights
    driver.b()
      .onTrue(new SetTwinklePattern(normalLights, LightsConstants.BRGColors.get("black"), LightsConstants.BRGColors.get("yellow"), 2));
    driver.a()
      .onTrue(new ResetLED(normalLights));

    //turret
    // operator.a()
      // .onTrue(new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.TEST)))
      // .onFalse(new InstantCommand(() -> turret.setWantedTurretState(TurretWantedState.IDLE)));

    //shooter
    // operator.b()
      // .onTrue(new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.TEST)))
      // .onFalse(new InstantCommand(() -> shooter.setWantedShooterState(ShooterWantedState.IDLE)));

    /* UNNEEDED, DELETE */
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    // final var idle = new SwerveRequest.Idle();
    // RobotModeTriggers.disabled().whileTrue(
    //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    // );
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }

  public void configureAutoCommands() {
    sendableChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("autos", sendableChooser);
  }

}

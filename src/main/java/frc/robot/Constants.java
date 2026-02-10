// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.PolynomialRegression;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static Optional<Alliance> alliance = DriverStation.getAlliance();
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ShooterConstants {
    public static double activeWaitingSpeed;
    public static double inactiveWaitingSpeed;
    public static PolynomialRegression hoodAngleInterpolation;
    public static PolynomialRegression shooterSpeedInterpolation;
    public static double distanceToHub;
    public static double passDistance;

    public static int shooterMotionMagicExpoK_V;
    public static int shooterMotionMagicExpoK_A;
    public static int shooterMotionMagicAccel;
    public static int shooterMotionMagicJerk;

    public static int SupplyCurrentLimit;
    public static int StatorCurrentLimit;

    public static int hoodMotorID;
    public static int shooterMotor1ID;
    public static int shooterMotor2ID;

    public static double[] shooterPID = {0, 0, 0};
    public static double[] shooterSVA = {0, 0, 0};
    public enum ShooterWantedState {
      IDLE,
      WAIT,
      PASS_SHOOT,
      HUB_SHOOT

    }
    public enum SystemState {
      IDLING,
      ACTIVE_WAITING,
      INACTIVE_WAITING,
      PASS_SHOOTING,
      HUB_SHOOTING
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Vector;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.PolynomialRegression;
// Lights
import java.util.Map;
import static edu.wpi.first.units.Units.Meters;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;

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
    public static final int kOperatorControllerPort = 1;
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

    public static int SupplyCurrentLimit = 80;
    public static int StatorCurrentLimit = 80;

    public static int hoodMotorID = 53;
    public static int shooterMotor1ID = 61;
    public static int shooterMotor2ID = 60;

    public static double[] shooterPID = {0, 0, 0};
    public static double[] shooterSVA = {0, 0, 0};
    public enum ShooterWantedState {
      IDLE,
      WAIT,
      PASS_SHOOT,
      HUB_SHOOT,
      TEST
    }
    public enum SystemState {
      IDLING,
      ACTIVE_WAITING,
      INACTIVE_WAITING,
      PASS_SHOOTING,
      HUB_SHOOTING,
      TESTING
    }
  }

  public static class IntakeConstants {
    public static double intakeMotionMagicExpoK_V = 0.13;
    public static double intakeMotionMagicExpoK_A = 0.1;
    public static double intakeMotionMagicAccel;
    public static double intakeMotionMagicCruiseVel;

    public static int SupplyCurrentLimit = 80;
    public static int StatorCurrentLimit = 80;

    public static int intakeMotorID = 32;
    public static int intakeExtensionMotorID = 31;

    public static double intakingMAXPosition = 10.022461;
    public static double intakingPosition = 10;
    public static double intakingSpeed = -0.5;
    public static double shootingPosition;
    public static double retractingPos = 0;
    public static double[] intakePID = {0.3, 0, 0};
    public static double[] intakeSVA = {0, 0.13, 0.01};
    public enum IntakeWantedState {
      IDLE,
      INTAKE,
      SHOOT, 
      RETRACT,
      RESET,
      SCORE
    }
    public enum SystemState {
      IDLING,
      INTAKING,
      SHOOTING,
      RETRACTING,
      RESETING,
      SCORING
      
    }
  }  
  
  
  public static class TurretConstants {
    public static int turretMotionMagicExpoK_V;
    public static int turretMotionMagicExpoK_A;
    public static int turretMotionMagicAccel;
    public static int turretMotionMagicJerk;

    public static int SupplyCurrentLimit = 80;
    public static int StatorCurrentLimit = 80;

    public static int turretMotorID = 50;
    public static int encoderID = 54;

    public static int passAimPosition;
    public static int hubAimPosition;
    public static int trenchPresetPosition;

    public static double[] turretPID = {0, 0, 0};
    public static double[] turretSVA = {0, 0, 0};

    public enum TurretWantedState {
      IDLE,
      AIM,
      TRENCH_PRESET,
      CLOSE_PRESET,
      TEST
      
    }
    public enum SystemState {
      IDLING,
      PASS_AIMING,
      HUB_AIMING,
      TRENCH_PRESETTING,
      CLOSE_PRESETTING,
      TESTING
    }
  }

  public static class FeederConstants {
    public static int feederMotionMagicExpoK_V;
    public static int feederMotionMagicExpoK_A;
    public static int feederMotionMagicAccel;
    public static int feederMotionMagicJerk;
    
    public static int SupplyCurrentLimit = 80;
    public static int StatorCurrentLimit = 80;
    
    public static int towerMotorID = 41;
    public static int spindexerMotorID = 40;

    public static double feederIntakeSpeed;
    public static double feederShootSpeed;
    
    public static double[] feederPID = {0, 0, 0};
    public static double[] feederSVA = {0, 0, 0};
    
    public enum FeederWantedState {
      IDLE,
      INTAKE,
      SHOOT,
      FEEDTEST
    }
    public enum SystemState {
      IDLING,
      INTAKING,
      SHOOTING,
      FEEDTESTING
    }
  }

  public static class LightsConstants {
    public static Distance spacing = Meters.of(1 / 60);   // (1 / 60) - 60 leds per 1m strip [Spacing: 1m/#ofLEDs]
    // Main LED Strip (sides)
    public static int main_port = 0;
    public static int main_length = 48;   // 48, 24 a side
    public static int main_brightness = 50;
    // Signal LED Sector (on shooter)
    //public static int signal_port = None;
    //public static int signal_length = 10;   // 10, 5 a side

    public static enum LightsType {
        ENDGAME,
        CLIMB,
        SHOOTING,
        INTAKE,
        IDLE,
        DISABLED
      }

    public static class Colors {
        public static int[] RED = new int[] { 255, 0, 0 };
        public static int[] GREEN = new int[] { 0, 255, 0 };
        public static int[] BLUE = new int[] { 0, 0, 255 };
        public static int[] GOLD = new int[] { 175, 184, 6 };
        public static int[] MAGENTA = new int[] { 255, 0, 255 };
        public static int[] BRIGHT = new int[] { 234, 255, 48 };
      }

    // RGB Color Map
    // Not needed: use Color.k[colorname]
    public static Map<String, Color> RGBColors = Map.ofEntries(
        Map.entry("black", new Color(0, 0, 0)),
        Map.entry("white", new Color(255, 255, 255)),
        Map.entry("red", new Color(255, 0, 0)),
        Map.entry("green", new Color(0, 255, 0)),
        Map.entry("blue", new Color(0, 0, 255)),
        Map.entry("gold", new Color(175, 184, 6)),
        Map.entry("team_Gold", new Color(179, 134, 27)),
        Map.entry("yellow", new Color(255, 255, 0)),
        Map.entry("orange", new Color(255, 165, 0)),
        Map.entry("pink", new Color(255, 20, 147)),
        Map.entry("magenta", new Color(255, 0, 255)),
        Map.entry("bright", new Color(234, 255, 48)));

    // GRB Color Map (Old LEDs)
    public static Map<String, Color> GRBColors = Map.ofEntries(
        Map.entry("black", new Color(0, 0, 0)),
        Map.entry("white", new Color(255, 255, 255)),
        Map.entry("red", new Color(0, 255, 0)),
        Map.entry("green", new Color(255, 0, 0)),
        Map.entry("blue", new Color(0, 0, 255)),
        Map.entry("gold", new Color(184, 175, 6)),
        Map.entry("team_Gold", new Color(134, 179, 27)),
        Map.entry("yellow", new Color(255, 255, 0)),
        Map.entry("orange", new Color(165, 255, 0)),
        Map.entry("pink", new Color(20, 255, 147)),
        Map.entry("magenta", new Color(0, 255, 255)),
        Map.entry("bright", new Color(255, 234, 48)));

    // GBR Color Map
    public static Map<String, Color> GBRColors = Map.ofEntries(
        Map.entry("black", new Color(0, 0, 0)),
        Map.entry("white", new Color(255, 255, 255)),
        Map.entry("red", new Color(0, 0, 255)),
        Map.entry("green", new Color(255, 0, 0)),
        Map.entry("blue", new Color(0, 255, 0)),
        Map.entry("gold", new Color(184, 6, 175)),
        Map.entry("team_Gold", new Color(134, 27, 179)),
        Map.entry("yellow", new Color(255, 0, 255)),
        Map.entry("orange", new Color(165, 0, 255)),
        Map.entry("pink", new Color(20, 147, 255)),
        Map.entry("magenta", new Color(0, 255, 255)),
        Map.entry("bright", new Color(255, 48, 234)));

    // BRG Color Map (New LED strip)
    public static Map<String, Color> BRGColors = Map.ofEntries(
        Map.entry("black", new Color(0, 0, 0)),                 // (0,0,0) → (0,0,0)
        Map.entry("white", new Color(255, 255, 255)),           // (255,255,255) → (255,255,255)
        Map.entry("red", new Color(0, 255, 0)),                 // RGB(255,0,0) → BRG(0,255,0)
        Map.entry("green", new Color(0, 0, 255)),               // RGB(0,255,0) → BRG(0,0,255)
        Map.entry("blue", new Color(255, 0, 0)),                // RGB(0,0,255) → BRG(255,0,0)
        Map.entry("gold", new Color(6, 175, 184)),              // RGB(175,184,6) → BRG(6,175,184)
        Map.entry("team_Gold", new Color(27, 179, 134)),        // RGB(179,134,27) → BRG(27,179,134)
        Map.entry("yellow", new Color(0, 255, 255)),            // RGB(255,255,0) → BRG(0,255,255)
        Map.entry("orange", new Color(0, 255, 165)),            // RGB(255,165,0) → BRG(0,255,165)
        Map.entry("pink", new Color(147, 255, 20)),             // RGB(255,20,147) → BRG(147,255,20)
        Map.entry("magenta", new Color(255, 255, 0)),           // RGB(255,0,255) → BRG(255,255,0)
        Map.entry("bright", new Color(48, 234, 255)));          // RGB(234,255,48) → BRG(48,234,255)
  }

  public static class VisionConstants {
    public static Transform2d turretToCenter = 
      new Transform2d(
        Units.inchesToMeters(-6.5), 
        Units.inchesToMeters(-6), 
        new Rotation2d());

    public static Translation2d RED_HUB_POSE =
      new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(158.84));
    public static Translation2d BLUE_HUB_POSE;

    public static double bumperToBumper; // inches

    public static Transform3d kRobotToCam = new Transform3d(
      new Translation3d(
        -(Units.inchesToMeters(0.643)), 
        Units.inchesToMeters(0.616), 
        Units.inchesToMeters(17.467)),
      new Rotation3d(
        0, 
        Units.degreesToRadians(23), 
        0)
      );

    public static Transform3d kRobotToCam2 = new Transform3d(
      new Translation3d(
        -(Units.inchesToMeters(12.889)), 
        Units.inchesToMeters(0.836),
        Units.inchesToMeters(9.317)),
      new Rotation3d(
        0, 
        Units.degreesToRadians(20), 
        Units.degreesToRadians(45))
        );

    public static Transform3d kRobotToCam3 = new Transform3d(
      new Translation3d(
        -(Units.inchesToMeters(12.888)), 
        -(Units.inchesToMeters(12.837)),
        Units.inchesToMeters(9.378)),
      new Rotation3d(
        0, 
        Units.degreesToRadians(20), 
        Units.degreesToRadians(135))
        );

    public static String cameraName = "camera1";
    public static String camera2Name = "camera2";
    public static String camera3Name = "camera3";

    /* standard deviations for vision calculations */
    public static edu.wpi.first.math.Vector<N3> kSingleTagStdDevs = VecBuilder.fill(2, 2, 2);
    public static edu.wpi.first.math.Vector<N3> kMultiTagStdDevs = VecBuilder.fill(1, 1, 1);
    public static edu.wpi.first.math.Vector<N3> odoStdDEvs = VecBuilder.fill(.2, .2, .2);
    public static double odometryUpdateFrequency = 250;
  }
}

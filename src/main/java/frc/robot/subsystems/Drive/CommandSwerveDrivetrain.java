package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants.ScoringZone;
import frc.robot.subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.Scoring.ShotCalc;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    public Vision vision = new Vision();
    private Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
    private Field2d TheField = new Field2d();
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    public Pose2d getPose() {
        return this.getState().Pose;
    }

//    private final StructArrayPublisher<Pose3d> trajectoryPublisher = 
//     NetworkTableInstance.getDefault()
//         .getTable("SmartDashboard")
//         .getStructArrayTopic("ball trajectory 3d", Pose3d.struct)
//         .publish();

    public Translation2d getScoringLocation() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance.equals(Alliance.Blue)) {
            if (0 < getPose().getX() && getPose().getX() < 4.6) {
                return FieldConstants.BLUE_HUB_POSE;
            } else {
                if (4.03 < getPose().getY()) {
                    return FieldConstants.BLUE_PASS_SPOT_2;
                } else {
                    return FieldConstants.BLUE_PASS_SPOT_1;
                }
            }
        } else {
            if (11.9 < getPose().getX() && getPose().getX() < 16.6) {
                return FieldConstants.RED_HUB_POSE;
            } else {
                if (4.03 < getPose().getY()) {
                    return FieldConstants.RED_PASS_SPOT_2;
                } else {
                    return FieldConstants.RED_PASS_SPOT_1;
                }
            }
        }
    }

    public Pose2d getTurretPose() {
        Pose2d turretPose = getPose().transformBy(VisionConstants.turretToCenter);
        turretPose = turretPose.rotateAround(turretPose.getTranslation(), Rotation2d.k180deg);

        double timeOfFlight = ShooterConstants.timeOfFlightInterpolation.getPrediction(getDistanceFromHub());
        double xSpeed = getState().Speeds.vxMetersPerSecond;
        double ySpeed = getState().Speeds.vyMetersPerSecond;
        double rotSpeed = getState().Speeds.omegaRadiansPerSecond;
        ChassisSpeeds fieldCentric = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                getCurrentTurretPose().getRotation());
        double futurePoseX = turretPose.getX() + fieldCentric.vxMetersPerSecond * timeOfFlight;
        double futurePoseY = turretPose.getY() + fieldCentric.vyMetersPerSecond * timeOfFlight;
        double futurePoseRot = turretPose.getRotation().getRadians()
                + fieldCentric.omegaRadiansPerSecond * timeOfFlight;
        Pose2d futurePose = new Pose2d(futurePoseX, futurePoseY, Rotation2d.fromRadians(futurePoseRot));

        TheField.getObject("turret pose").setPose(turretPose);
        TheField.getObject("turret pose").setPose(futurePose);

        return futurePose;
    }
    
    private final StructArrayPublisher<Pose3d> trajectoryPublisher =
    NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getStructArrayTopic("ball trajectory 3d", Pose3d.struct)
        .publish();

private void updateBallTrajectory() {
    if (!Robot.isSimulation()) return;

    Translation2d turretPos = getCurrentTurretPose().getTranslation();
    double distance = getDistanceFromHub();
    double tof = ShooterConstants.timeOfFlightInterpolation.getPrediction(distance);

    double hoodAngleDeg = ShooterConstants.HOOD_MAP.get(distance)
        * ShooterConstants.hoodConversionRotToDeg;
    double hoodHomeAngle = 18.0;
    double launchAngleRad = Math.toRadians(90.0 - (hoodHomeAngle + hoodAngleDeg));

    double flywheelSurfaceSpeed = currentShotCommand.RPS() * Math.PI * (4 * 0.0254);
    double exitSpeed = flywheelSurfaceSpeed * 0.45;
    double horizontalSpeed = exitSpeed * Math.cos(launchAngleRad);
    double verticalSpeed = exitSpeed * Math.sin(launchAngleRad);

    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        getState().Speeds, getPose().getRotation());
    Translation2d virtualTarget = getScoringLocation().minus(
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond)
        .times(tof));

    double dx = virtualTarget.getX() - turretPos.getX();
    double dy = virtualTarget.getY() - turretPos.getY();
    double horizontalDist = Math.sqrt(dx * dx + dy * dy);
    if (horizontalDist < 0.01) return;

    double vx = (dx / horizontalDist) * horizontalSpeed;
double vy = (dy / horizontalDist) * horizontalSpeed;

    double launchHeight = 0.46;
    double g = 9.81;
    int steps = 20;
    double[] arr = new double[steps * 3];
    int count = 0;

    // THIS LOOP WAS MISSING - fills arr with trajectory points
   // Use time to hit ground, not TOF to goal
double timeToGround = (verticalSpeed + Math.sqrt(verticalSpeed * verticalSpeed + 2 * g * launchHeight)) / g;

for (int i = 0; i < steps; i++) {
    double t = (timeToGround / steps) * i;
    double x = turretPos.getX() + vx * t;
    double y = turretPos.getY() + vy * t;
    double z = launchHeight + verticalSpeed * t - 0.5 * g * t * t;
    if (z < 0) break;
    arr[count * 3]     = x;
    arr[count * 3 + 1] = y;
    arr[count * 3 + 2] = z;
    count++;
}

    // Build Pose3d array and publish
    Pose3d[] poseArray = new Pose3d[count];
    for (int i = 0; i < count; i++) {
        poseArray[i] = new Pose3d(
            arr[i * 3],
            arr[i * 3 + 1],
            arr[i * 3 + 2],
            new Rotation3d()
        );
    }

  trajectoryPublisher.set(poseArray);

    // Debug
    SmartDashboard.putNumber("Trajectory VX", vx);
    SmartDashboard.putNumber("Trajectory VY", vy);
    SmartDashboard.putNumber("Trajectory Vertical Speed", verticalSpeed);
    SmartDashboard.putNumber("Trajectory TOF", tof);
    SmartDashboard.putNumber("Trajectory Distance", distance);
    SmartDashboard.putNumber("Trajectory Exit Speed", exitSpeed);
    SmartDashboard.putNumber("Trajectory Horizontal Speed", horizontalSpeed);
    SmartDashboard.putNumber("Trajectory Point Count", count);
    // Debug - verify virtual target matches shot angle
double angleToVirtualTarget = Math.toDegrees(Math.atan2(
    virtualTarget.getY() - turretPos.getY(),
    virtualTarget.getX() - turretPos.getX()));
SmartDashboard.putNumber("Angle to Virtual Target", angleToVirtualTarget);
SmartDashboard.putNumber("Shot Command Angle", currentShotCommand.turretAngle().getDegrees());
SmartDashboard.putNumber("Virtual Target X", virtualTarget.getX());
SmartDashboard.putNumber("Virtual Target Y", virtualTarget.getY());
SmartDashboard.putNumber("Turret Pos X", turretPos.getX());
SmartDashboard.putNumber("Turret Pos Y", turretPos.getY());
}

    private ChassisSpeeds filteredFieldSpeeds = new ChassisSpeeds(0, 0, 0);

    public ShotCalc.ShooterCommand currentShotCommand =
        new ShotCalc.ShooterCommand(0, new Rotation2d(), 0);

    public Pose2d getCurrentTurretPose() {
        Pose2d turretPose = getPose().transformBy(VisionConstants.turretToCenter);
        turretPose = turretPose.rotateAround(turretPose.getTranslation(), Rotation2d.k180deg);
        return turretPose;
    }

    public Translation2d getHub() {
        Translation2d goalLocation = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? FieldConstants.RED_HUB_POSE
                : FieldConstants.BLUE_HUB_POSE;
        return goalLocation;
    }

    public double getDistanceFromHub() {
        double distance = Math.hypot(getXfromHub(), getYfromHub());
        return distance;
    }

    public double getXfromLocation(Pose2d target) {
        double xDistance;
        Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (allianceColor == Alliance.Red) {
            xDistance = Math.abs(target.getX() - getCurrentTurretPose().getX());
        } else {
            xDistance = Math.abs(target.getX() - getCurrentTurretPose().getX());
        }
        return xDistance;
    }

    public double getYfromLocation(Pose2d target) {
        double yDistance;
        Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (allianceColor == Alliance.Red) {
            yDistance = Math.abs(target.getY() - getCurrentTurretPose().getY());
        } else {
            yDistance = Math.abs(target.getY() - getCurrentTurretPose().getY());
        }
        return yDistance;
    }

    public double getXfromHub() {
        double xDistance;
        Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (allianceColor == Alliance.Red) {
            xDistance = FieldConstants.RED_HUB_POSE.getX() - getCurrentTurretPose().getX();
        } else {
            xDistance = FieldConstants.BLUE_HUB_POSE.getX() - getCurrentTurretPose().getX();
        }
        return xDistance;
    }

    public double getYfromHub() {
        double yDistance;
        Alliance allianceColor = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (allianceColor == Alliance.Red) {
            yDistance = FieldConstants.RED_HUB_POSE.getY() - getCurrentTurretPose().getY();
        } else {
            yDistance = FieldConstants.BLUE_HUB_POSE.getY() - getCurrentTurretPose().getY();
        }
        return yDistance;
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose,
                    this::resetPose,
                    () -> getState().Speeds,
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            new PIDConstants(10, 0, 0),
                            new PIDConstants(7, 0, 0)),
                    config,
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6).per(Second),
                    Volts.of(Math.PI),
                    null,
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("The field", TheField);
        configureAutoBuilder();
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public double shotCommandTimestamp = 0;

    @Override
    public void periodic() {
        if (DriverStation.getAlliance().isPresent()) {
            ChassisSpeeds rawFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    getState().Speeds,
                    getPose().getRotation());

            shotCommandTimestamp = Timer.getFPGATimestamp();
            currentShotCommand = ShotCalc.calculateSOTF(
                    getPose().getTranslation(),
                    getCurrentTurretPose().getTranslation(),
                    rawFieldSpeeds,
                    getState().Speeds.omegaRadiansPerSecond,
                    getScoringLocation());
        }

        // Publish clean 50Hz pose for simulation visualization
if (Robot.isSimulation()) {
    SmartDashboard.putNumberArray("SimRobotPose", new double[]{
        getPose().getX(),
        getPose().getY(),
        getPose().getRotation().getRadians()
    });
}

if (Robot.isSimulation()) {
    updateBallTrajectory();
}

        // Update field visualization
        TheField.getObject("robot").setPose(getPose());
        TheField.getObject("target").setPose(new Pose2d(getScoringLocation(), new Rotation2d()));
        TheField.getObject("turret").setPose(getCurrentTurretPose());

        if (DriverStation.getAlliance().isPresent()) {
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    getState().Speeds, getPose().getRotation());
            double tof = ShooterConstants.timeOfFlightInterpolation.getPrediction(getDistanceFromHub());
            TheField.getObject("virtual target").setPose(
                    new Pose2d(
                            getScoringLocation().minus(
                                    new Translation2d(
                                            fieldSpeeds.vxMetersPerSecond,
                                            fieldSpeeds.vyMetersPerSecond)
                                            .times(tof)),
                            new Rotation2d()));
        }

        // SmartDashboard logging
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Rotation", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Distance From Hub", getDistanceFromHub());
        SmartDashboard.putNumber("Field Vx", ChassisSpeeds.fromRobotRelativeSpeeds(
                getState().Speeds, getPose().getRotation()).vxMetersPerSecond);
        SmartDashboard.putNumber("Field Vy", ChassisSpeeds.fromRobotRelativeSpeeds(
                getState().Speeds, getPose().getRotation()).vyMetersPerSecond);
        SmartDashboard.putNumber("Omega Rad", getState().Speeds.omegaRadiansPerSecond);
        

        // Operator perspective
        if (Robot.isSimulation()) {
            setOperatorPerspectiveForward(kBlueAlliancePerspectiveRotation);
            m_hasAppliedOperatorPerspective = true;
        } else if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}
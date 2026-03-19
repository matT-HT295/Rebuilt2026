package frc.robot.subsystems.Scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShooterConstants;

public final class ShotCalc {

    public static record ShooterCommand(double RPS, Rotation2d turretAngle, double hoodAngle) {
    }

    public static ShooterCommand calculateSOTF(
            Translation2d robotPosition,
            ChassisSpeeds chassisSpeeds,
            Translation2d goalPosition,
            double latencyCompensation) {

        // 1. Predict future robot position
        Translation2d robotVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond);
        Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));

        // 2. Target vector & distance
        Translation2d toGoal = goalPosition.minus(futurePos);
        double distance = toGoal.getNorm();
        if (distance < 1e-6) {
            // Too close, return safe defaults
            return new ShooterCommand(0, new Rotation2d(), 0);
        }
        Translation2d targetDirection = toGoal.div(distance);

        // 3. Lookup baseline values from tables
        double baselineTOF = ShooterConstants.TOF_MAP.get(distance);
        double baselineRPS = ShooterConstants.RPS_MAP.get(distance);
        double baselineHoodAngle = ShooterConstants.HOOD_MAP.get(distance);

        double baselineVelocity = distance / baselineTOF;

        // 4. Desired shot velocity vector
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // 5. Compensate for robot motion
        Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

        // 6. Turret angle points along shot vector
        Rotation2d turretAngle = shotVelocity.getAngle();
        double requiredVelocity = shotVelocity.getNorm();

        // 7. Scale RPS proportionally to velocity change
        double velocityRatio = requiredVelocity / baselineVelocity;
        double adjustedRPS = baselineRPS * velocityRatio;
        adjustedRPS = MathUtil.clamp(adjustedRPS, ShooterConstants.MIN_RPS, ShooterConstants.MAX_RPS);

        // 8. Adjust hood slightly to maintain trajectory (optional small correction)
        // Keep hood mostly from baseline, small adjustment for horizontal component
        double adjustedHood = baselineHoodAngle; // can refine if needed

        return new ShooterCommand(adjustedRPS, turretAngle, adjustedHood);
    }
}
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class MegaTrackCommand extends Command {
  RobotContainer m_RobotContainer;
  Translation2d m_targetTranslation2d;

  public MegaTrackCommand(RobotContainer _RobotContainer, Translation2d targetTranslation) {
    m_RobotContainer = _RobotContainer;
    m_targetTranslation2d = targetTranslation;
  }

  @Override
  public void initialize() {}

  Translation2d getFixedTarget(
      Translation2d robotTranslation2d,
      Translation2d targetTranslation2d,
      ChassisSpeeds fieldRelativeRobotSpeeds) {
    Translation2d m_robotTelativeTargetTranslation = targetTranslation2d.minus(robotTranslation2d);
    Translation2d m_velocityContributionTranslation =
        new Translation2d(
                fieldRelativeRobotSpeeds.vxMetersPerSecond,
                fieldRelativeRobotSpeeds.vyMetersPerSecond)
            .times(AutoShootConstants.FlyTime);
    Translation2d fixedTranslation2d =
        m_robotTelativeTargetTranslation.minus(m_velocityContributionTranslation);
    return fixedTranslation2d;
  }

  @Override
  public void execute() {
    Translation2d fixedTargetTranslation2d =
        getFixedTarget(
            m_RobotContainer.drive.getPose().getTranslation(),
            m_targetTranslation2d,
            m_RobotContainer.drive.getFieldRelativeSpeeds());
    Logger.recordOutput(
        "AutoShoot/FixedTarget",
        fixedTargetTranslation2d.plus(m_RobotContainer.drive.getPose().getTranslation()));
    double distanceToTarget = fixedTargetTranslation2d.getNorm();
    Rotation2d targetHeading = fixedTargetTranslation2d.getAngle();
    Logger.recordOutput(
        "AutoShoot/TargetPose",
        new Pose2d(m_RobotContainer.drive.getPose().getTranslation(), targetHeading));

    double hoodAngle = AutoShootConstants.hoodAngleMap.get(distanceToTarget);
    double shooterSpeed = AutoShootConstants.shooterSpeedMap.get(distanceToTarget);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}

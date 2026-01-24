package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

/**
 * Moving-shot compensation using fixed-point iteration on flight time.
 *
 * <p>Solves for T such that:
 *
 * <pre>
 * r(T) = g - p - T*v
 * d(T) = |r(T)|
 * T = flightTimeMap(d(T))
 * </pre>
 *
 * using fixed-point iteration each cycle, then uses r(T*) for heading/distance.
 */
public class MegaTrackIterativeCommand extends Command {
  private static final String LOG_PREFIX = "AutoShootIter";

  // Heading control
  private static final double HEADING_KP = 6.0;
  private static final double HEADING_KI = 0.0;
  private static final double HEADING_KD = 0.1;

  // Fixed-point iteration tuning
  private static final int MAX_ITERS = 6;
  private static final double EPS_T_SEC = 1e-3;
  private static final double RELAX = 0.7; // 1.0 = plain fixed-point, <1 adds damping
  private static final double MIN_T_SEC = 0.0;
  private static final double MAX_T_SEC = 2.0;

  // Shoot gating
  private static final double MIN_SHOOT_DIST_METERS = 0.8;
  private static final double MAX_SHOOT_DIST_METERS = 5.8;
  private static final double HEADING_TOL_RAD = Math.toRadians(2.0);
  private static final double FLYWHEEL_RPS_TOL = 1.5;
  private static final double HOOD_DEG_TOL = 1.0;
  private static final double FEEDER_RPS = 20.0;
  private static final double INDEXER_VOLTS = 10.0;

  private final RobotContainer robot;
  private final frc.robot.subsystems.drive.Drive drive;
  private final Translation2d targetTranslation;
  private final java.util.function.DoubleSupplier xSupplier;
  private final java.util.function.DoubleSupplier ySupplier;

  private final PIDController headingController =
      new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

  private double lastFlightTimeSec = AutoShootConstants.FlyTime;
  private Rotation2d heldHeading = new Rotation2d();

  public MegaTrackIterativeCommand(RobotContainer robot, Translation2d targetTranslation) {
    this.robot = robot;
    this.drive = robot.drive;
    this.targetTranslation = targetTranslation;
    this.xSupplier = robot.getDriveXSupplier();
    this.ySupplier = robot.getDriveYSupplier();
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(robot.drive, robot.shooter, robot.feeder, robot.indexer);
  }

  private Translation2d getShotOriginField() {
    Pose2d pose = drive.getPose();
    Translation2d offsetRobot =
        new Translation2d(
            Constants.shooterConstants.FLYWHEEL_OFFSET_X_METERS,
            Constants.shooterConstants.FLYWHEEL_OFFSET_Y_METERS);
    Translation2d offsetField = offsetRobot.rotateBy(pose.getRotation());
    return pose.getTranslation().plus(offsetField);
  }

  /** Returns r(T) = g - p - T*v (all in field frame). */
  private Translation2d compensatedVector(
      Translation2d shotOrigin, ChassisSpeeds vField, double tSec) {
    return targetTranslation
        .minus(shotOrigin)
        .minus(new Translation2d(vField.vxMetersPerSecond, vField.vyMetersPerSecond).times(tSec));
  }

  private double iterateFlightTimeSec(Translation2d shotOrigin, ChassisSpeeds vField) {
    double t = MathUtil.clamp(lastFlightTimeSec, MIN_T_SEC, MAX_T_SEC);

    // If last T is invalid/uninitialized, seed with map(distance at T=0)
    if (!Double.isFinite(t)) {
      double d0 = targetTranslation.minus(shotOrigin).getNorm();
      t = AutoShootConstants.flightTimeMap.get(d0);
    }

    for (int i = 0; i < MAX_ITERS; i++) {
      Translation2d r = compensatedVector(shotOrigin, vField, t);
      double d = r.getNorm();
      double tNext = AutoShootConstants.flightTimeMap.get(d);
      tNext = MathUtil.clamp(tNext, MIN_T_SEC, MAX_T_SEC);

      double tNew = (1.0 - RELAX) * t + RELAX * tNext;
      Logger.recordOutput(LOG_PREFIX + "/Iter/T_" + i, t);
      Logger.recordOutput(LOG_PREFIX + "/Iter/D_" + i, d);
      Logger.recordOutput(LOG_PREFIX + "/Iter/TNext_" + i, tNext);

      if (Math.abs(tNew - t) < EPS_T_SEC) {
        t = tNew;
        break;
      }
      t = tNew;
    }

    lastFlightTimeSec = t;
    return t;
  }

  @Override
  public void initialize() {
    headingController.reset();
    heldHeading = drive.getRotation();
  }

  @Override
  public void execute() {
    Translation2d shotOrigin = getShotOriginField();
    ChassisSpeeds vField = drive.getFieldRelativeSpeeds();

    double tSec = iterateFlightTimeSec(shotOrigin, vField);
    Translation2d r = compensatedVector(shotOrigin, vField, tSec);
    double dist = r.getNorm();

    // Heading target comes from compensated vector (if too close, just hold current).
    if (dist > 1e-3) {
      heldHeading = r.getAngle();
    }

    // Driver translation (field-relative)
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Heading control + simple omega FF using rDot=-v
    double omegaPid =
        headingController.calculate(drive.getRotation().getRadians(), heldHeading.getRadians());
    double den = Math.max(r.getNorm() * r.getNorm(), 0.40 * 0.40);
    double omegaFf =
        (r.getX() * (-vField.vyMetersPerSecond) - r.getY() * (-vField.vxMetersPerSecond)) / den;
    double omega = omegaPid + omegaFf;
    omega =
        MathUtil.clamp(
            omega, -drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec());

    ChassisSpeeds fieldRelative =
        new ChassisSpeeds(
            linearVelocity.getX() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            linearVelocity.getY() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelative,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    // Shooter setpoints based on compensated distance
    double shooterRps = AutoShootConstants.shooterSpeedMap.get(dist);
    double hoodDeg = AutoShootConstants.hoodAngleMap.get(dist);
    robot.shooter.setVelocity(shooterRps);
    robot.shooter.setHoodAngle(hoodDeg);

    // Shoot gating (same idea as MegaTrackCommand)
    boolean distOk = dist >= MIN_SHOOT_DIST_METERS && dist <= MAX_SHOOT_DIST_METERS;
    double flywheelErr = shooterRps - robot.shooter.getFlywheelVelocityRps();
    double hoodErr = hoodDeg - robot.shooter.getHoodAngleDeg();
    double headingErrRad =
        MathUtil.angleModulus(heldHeading.getRadians() - drive.getRotation().getRadians());

    boolean ready =
        distOk
            && Math.abs(flywheelErr) <= FLYWHEEL_RPS_TOL
            && Math.abs(hoodErr) <= HOOD_DEG_TOL
            && Math.abs(headingErrRad) <= HEADING_TOL_RAD;

    if (ready) {
      robot.feeder.setVelocity(FEEDER_RPS);
      robot.indexer.setVoltage(INDEXER_VOLTS);
    } else {
      robot.feeder.stop();
      robot.indexer.stop();
    }

    // Logs
    Logger.recordOutput(LOG_PREFIX + "/ShotOrigin", shotOrigin);
    Logger.recordOutput(LOG_PREFIX + "/T", tSec);
    Logger.recordOutput(LOG_PREFIX + "/R", r);
    Logger.recordOutput(LOG_PREFIX + "/Dist", dist);
    Logger.recordOutput(LOG_PREFIX + "/HeldHeadingDeg", heldHeading.getDegrees());
    Logger.recordOutput(LOG_PREFIX + "/OmegaPID", omegaPid);
    Logger.recordOutput(LOG_PREFIX + "/OmegaFF", omegaFf);
    Logger.recordOutput(LOG_PREFIX + "/Ready", ready);
    Logger.recordOutput(LOG_PREFIX + "/FlywheelErrRps", flywheelErr);
    Logger.recordOutput(LOG_PREFIX + "/HoodErrDeg", hoodErr);
    Logger.recordOutput(LOG_PREFIX + "/HeadingErrDeg", Math.toDegrees(headingErrRad));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    robot.feeder.stop();
    robot.indexer.stop();
    robot.shooter.stop();
  }
}

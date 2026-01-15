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

public class MegaTrackCommand extends Command {
  private static final double HEADING_KP = 6.0;
  private static final double HEADING_KI = 0.0;
  private static final double HEADING_KD = 0.1;

  /** Scale for shooter acceleration feedforward computed from slope*dDot. */
  private static final double SHOOTER_ACCEL_FF_GAIN = 1.0;
  /** If true, include measured acceleration contribution in FF calculations (can be jumpy). */
  private static final boolean USE_ACCEL_IN_FF = false;

  // --- Shooting gating / heading hold tuning ---
  private static final double MIN_SHOOT_DIST_METERS = 0.8;
  private static final double MAX_SHOOT_DIST_METERS = 5.8;
  private static final double HEADING_HOLD_MIN_DIST_METERS = 0.9;
  private static final double HEADING_HOLD_MAX_DIST_METERS = 5.6;

  private static final double FLYWHEEL_RPS_TOL = 1.5;
  private static final double HOOD_DEG_TOL = 1.0;
  private static final double HEADING_TOL_RAD = Math.toRadians(2.0);

  private static final double FEEDER_RPS = 20.0;

  private final RobotContainer robotContainer;
  private final frc.robot.subsystems.drive.Drive drive;
  private final java.util.function.DoubleSupplier xSupplier;
  private final java.util.function.DoubleSupplier ySupplier;
  private final Translation2d targetTranslation;

  private final PIDController headingController =
      new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

  private Rotation2d heldHeading = new Rotation2d();

  /** Returns the flywheel/shot origin position in field coordinates (accounts for offset). */
  private Translation2d getShotOriginField() {
    var pose = drive.getPose();
    Translation2d offsetRobot =
        new Translation2d(
            Constants.shooterConstants.FLYWHEEL_OFFSET_X_METERS,
            Constants.shooterConstants.FLYWHEEL_OFFSET_Y_METERS);
    Translation2d offsetField = offsetRobot.rotateBy(pose.getRotation());
    return pose.getTranslation().plus(offsetField);
  }

  public MegaTrackCommand(RobotContainer robotContainer, Translation2d targetTranslation) {
    this.robotContainer = robotContainer;
    this.drive = robotContainer.drive;
    this.xSupplier = robotContainer.getDriveXSupplier();
    this.ySupplier = robotContainer.getDriveYSupplier();
    this.targetTranslation = targetTranslation;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive, robotContainer.shooter, robotContainer.feeder);
  }

  @Override
  public void initialize() {
    headingController.reset();
    heldHeading = drive.getRotation();
  }

  Translation2d getFixedTarget(
      Translation2d originTranslation2d,
      Translation2d targetTranslation2d,
      ChassisSpeeds fieldRelativeRobotSpeeds) {
    Translation2d m_robotTelativeTargetTranslation = targetTranslation2d.minus(originTranslation2d);
    Translation2d m_velocityContributionTranslation =
        new Translation2d(
                fieldRelativeRobotSpeeds.vxMetersPerSecond,
                fieldRelativeRobotSpeeds.vyMetersPerSecond)
            .times(AutoShootConstants.FlyTime);
    Translation2d fixedTranslation2d =
        m_robotTelativeTargetTranslation.minus(m_velocityContributionTranslation);
    return fixedTranslation2d;
  }

  private static double slopeAtDistance(
      edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap map, double d, double epsMeters) {

    double d0 = Math.max(0.0, d - epsMeters);
    double d1 = d + epsMeters;

    double y0 = map.get(d0);
    double y1 = map.get(d1);

    double denom = (d1 - d0);
    if (denom < 1e-6) return 0.0;
    return (y1 - y0) / denom; // units: (map-units)/meter
  }

  private static double estimateDistanceRate(
      Translation2d r, // aiming vector (field): g - p - T*v (or your fixedTarget vector)
      ChassisSpeeds vField, // field-relative velocity
      Translation2d aField, // field-relative acceleration
      double T) {
    double dist = r.getNorm();
    if (dist < 1e-6) return 0.0;

    double rHatX = r.getX() / dist;
    double rHatY = r.getY() / dist;

    // rDot: by default only use velocity term (accel term can be jumpy)
    double rdotX = -vField.vxMetersPerSecond;
    double rdotY = -vField.vyMetersPerSecond;
    if (USE_ACCEL_IN_FF) {
      // When r = g - p - T*v, then rDot = -(v + T*a)
      rdotX = -(vField.vxMetersPerSecond + T * aField.getX());
      rdotY = -(vField.vyMetersPerSecond + T * aField.getY());
    }

    return rHatX * rdotX + rHatY * rdotY; // m/s
  }

  /** Feedforward omega estimate to keep pointing at the target while translating/accelerating. */
  double rotateOmegaTarget(Translation2d robotAccelerationField) {
    final double T = AutoShootConstants.FlyTime;

    Translation2d robotPos = getShotOriginField();
    ChassisSpeeds vField = drive.getFieldRelativeSpeeds();

    double vx = vField.vxMetersPerSecond;
    double vy = vField.vyMetersPerSecond;

    double ax = robotAccelerationField.getX();
    double ay = robotAccelerationField.getY();

    // r = g - p - T*v
    double rx = targetTranslation.getX() - robotPos.getX() - T * vx;
    double ry = targetTranslation.getY() - robotPos.getY() - T * vy;

    // rDot = -(v + T*a) (optionally ignore accel term if jumpy)
    double rdotx = -vx;
    double rdoty = -vy;
    if (USE_ACCEL_IN_FF) {
      rdotx = -(vx + T * ax);
      rdoty = -(vy + T * ay);
    }

    // omega = (r x rDot) / |r|^2  (2D cross product scalar)
    double den = rx * rx + ry * ry;

    // guard against huge omega when very close or numerical noise
    final double minDist = 0.40; // meters, tune as needed
    den = Math.max(den, minDist * minDist);

    double omega = (rx * rdoty - ry * rdotx) / den; // rad/s

    // Optional clamp to avoid insane commands (tune to your drivetrain capability)
    final double maxOmega = 10.0; // rad/s
    omega = Math.max(-maxOmega, Math.min(maxOmega, omega));
    return omega;
  }

  @Override
  public void execute() {
    // Driver translation (field-relative)
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    Translation2d shotOrigin = getShotOriginField();

    // Compute target heading (field-relative) with fly-time compensation
    Translation2d fixedTargetTranslation2d =
        getFixedTarget(shotOrigin, targetTranslation, drive.getFieldRelativeSpeeds());
    Logger.recordOutput("AutoShoot/FixedTarget", fixedTargetTranslation2d.plus(shotOrigin));
    double distanceToTarget = fixedTargetTranslation2d.getNorm();

    // Update heading target only when distance is in a reasonable window.
    // Too close: heading jumps quickly; too far: we don't intend to shoot anyway.
    boolean allowHeadingUpdate =
        distanceToTarget >= HEADING_HOLD_MIN_DIST_METERS
            && distanceToTarget <= HEADING_HOLD_MAX_DIST_METERS
            && distanceToTarget > 1e-3;
    if (allowHeadingUpdate) {
      heldHeading = fixedTargetTranslation2d.getAngle();
    }

    Rotation2d targetHeading = heldHeading;
    Logger.recordOutput(
        "AutoShoot/TargetPose", new Pose2d(drive.getPose().getTranslation(), targetHeading));
    Logger.recordOutput("AutoShoot/ShotOrigin", shotOrigin);
    Logger.recordOutput("AutoShoot/HeadingAllowUpdate", allowHeadingUpdate);
    Logger.recordOutput("AutoShoot/HeadingHeldDeg", heldHeading.getDegrees());

    // Heading control (PID) to face the target
    double omegaPidRadPerSec =
        headingController.calculate(drive.getRotation().getRadians(), targetHeading.getRadians());

    double omegaFfRadPerSec = rotateOmegaTarget(drive.getFieldRelativeAcceleration());
    Logger.recordOutput("AutoShoot/HeadingOmegaPID", omegaPidRadPerSec);
    Logger.recordOutput("AutoShoot/HeadingOmegaFF", omegaFfRadPerSec);

    double omegaRadPerSec = omegaPidRadPerSec + omegaFfRadPerSec;
    omegaRadPerSec =
        MathUtil.clamp(
            omegaRadPerSec,
            -drive.getMaxAngularSpeedRadPerSec(),
            drive.getMaxAngularSpeedRadPerSec());

    // Combine into field-relative chassis speeds then convert to robot-relative for Drive
    ChassisSpeeds fieldRelative =
        new ChassisSpeeds(
            linearVelocity.getX() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            linearVelocity.getY() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            omegaRadPerSec);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelative,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    double d = distanceToTarget;

    // Baseline from maps (static)
    double hoodBase = AutoShootConstants.hoodAngleMap.get(d);
    double shooterBase = AutoShootConstants.shooterSpeedMap.get(d);

    // Slopes at current distance
    double hoodSlope = slopeAtDistance(AutoShootConstants.hoodAngleMap, d, 0.15); // deg/m or rad/m
    double shooterSlope =
        slopeAtDistance(AutoShootConstants.shooterSpeedMap, d, 0.15); // (rpm)/m or (rps)/m

    // Estimate delta distance over flight time due to motion
    ChassisSpeeds vField = drive.getFieldRelativeSpeeds();
    Translation2d aField = drive.getFieldRelativeAcceleration();

    // r = g - p - T*v  (same as your fixed target vector)
    Translation2d r =
        targetTranslation
            .minus(shotOrigin)
            .minus(
                new Translation2d(vField.vxMetersPerSecond, vField.vyMetersPerSecond)
                    .times(AutoShootConstants.FlyTime));

    double dDot = estimateDistanceRate(r, vField, aField, AutoShootConstants.FlyTime); // m/s

    // Feedforward signal:
    // - shooter: acceleration feedforward (RPS/s) = d(omega)/dd * dDot
    double shooterAccelFfRpsPerSec = shooterSlope * dDot * SHOOTER_ACCEL_FF_GAIN;

    Logger.recordOutput("AutoShoot/DistanceRate", dDot);
    Logger.recordOutput("AutoShoot/HoodSlope", hoodSlope);
    Logger.recordOutput("AutoShoot/ShooterSlope", shooterSlope);
    Logger.recordOutput("AutoShoot/ShooterAccelFFRpsPerSec", shooterAccelFfRpsPerSec);
    Logger.recordOutput("AutoShoot/ShooterAccelFFGain", SHOOTER_ACCEL_FF_GAIN);

    double hoodAngle = hoodBase;
    double shooterSpeed = shooterBase;

    Logger.recordOutput("AutoShoot/Distance", d);
    Logger.recordOutput("AutoShoot/HoodBase", hoodBase);
    Logger.recordOutput("AutoShoot/HoodFinal", hoodAngle);
    Logger.recordOutput("AutoShoot/ShooterBase", shooterBase);
    Logger.recordOutput("AutoShoot/ShooterFinal", shooterSpeed);

    // Actually command shooter for testing
    // Hood/backplate feedforward ignored for now
    robotContainer.shooter.setHoodAngle(hoodAngle);
    robotContainer.shooter.setVelocity(shooterSpeed, shooterAccelFfRpsPerSec);

    // --- Shoot readiness + feeder control ---
    boolean shootDistanceOk = d >= MIN_SHOOT_DIST_METERS && d <= MAX_SHOOT_DIST_METERS;

    double flywheelErr = shooterSpeed - robotContainer.shooter.getFlywheelVelocityRps();
    double hoodErr = hoodAngle - robotContainer.shooter.getHoodAngleDeg();
    double headingErrRad =
        MathUtil.angleModulus(targetHeading.getRadians() - drive.getRotation().getRadians());

    boolean flywheelOk = Math.abs(flywheelErr) <= FLYWHEEL_RPS_TOL;
    boolean hoodOk = Math.abs(hoodErr) <= HOOD_DEG_TOL;
    boolean headingOk = Math.abs(headingErrRad) <= HEADING_TOL_RAD;

    boolean readyToShoot = shootDistanceOk && flywheelOk && hoodOk && headingOk;
    Logger.recordOutput("AutoShoot/ShootDistanceOk", shootDistanceOk);
    Logger.recordOutput("AutoShoot/FlywheelErrRps", flywheelErr);
    Logger.recordOutput("AutoShoot/HoodErrDeg", hoodErr);
    Logger.recordOutput("AutoShoot/HeadingErrDeg", Math.toDegrees(headingErrRad));
    Logger.recordOutput("AutoShoot/ReadyToShoot", readyToShoot);

    if (readyToShoot) {
      robotContainer.feeder.setVelocity(FEEDER_RPS);
    } else {
      robotContainer.feeder.stop();
    }

    // Log suggested shooter parameters (for dashboards/other commands to consume)
    Logger.recordOutput("AutoShoot/DistanceToTarget", distanceToTarget);
    Logger.recordOutput(
        "AutoShoot/HoodAngleDeg", AutoShootConstants.hoodAngleMap.get(distanceToTarget));
    Logger.recordOutput(
        "AutoShoot/ShooterSpeedRPS", AutoShootConstants.shooterSpeedMap.get(distanceToTarget));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    robotContainer.shooter.stop();
    robotContainer.feeder.stop();
  }
}

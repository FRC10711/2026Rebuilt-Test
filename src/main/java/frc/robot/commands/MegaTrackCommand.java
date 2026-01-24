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
  private static final double HEADING_KD = 0.;

  /** Scale for shooter acceleration feedforward computed from slope*dDot. */
  private static final double SHOOTER_ACCEL_FF_GAIN = 1.0;
  /** If true, include measured acceleration contribution in FF calculations (can be jumpy). */
  private static final boolean USE_ACCEL_IN_FF = false;

  // --- Shooting gating / heading hold tuning ---
  private static final double MIN_SHOOT_DIST_METERS = 0.8;
  private static final double MAX_SHOOT_DIST_METERS = 5.8;
  private static final double HEADING_HOLD_MIN_DIST_METERS = 0.9;
  private static final double HEADING_HOLD_MAX_DIST_METERS = 5.6;

  private static final double FLYWHEEL_RPS_TOL = 1;
  private static final double HOOD_DEG_TOL = 1.;
  private static final double HEADING_TOL_RAD = Math.toRadians(2.0);

  private static final double FEEDER_RPS = 30.0;
  private static final double INDEXER_VOLTS = 10.0;

  // --- State machine thresholds ---
  /** Enter reaccelerate when flywheel drops below (setpoint - this). */
  private static final double REACCEL_ENTER_DROP_RPS = 30.0;
  /** Exit reaccelerate when flywheel rises above (setpoint - this). */
  private static final double REACCEL_EXIT_DROP_RPS = 1.0;
  /** Big acceleration feedforward applied during reaccelerate (RPS/s). */
  private static final double REACCEL_BIG_ACCEL_FF_RPS_PER_SEC = 0.0;

  private enum State {
    ALIGNING,
    SHOOT,
    REACCELERATE
  }

  private final RobotContainer robotContainer;
  private final frc.robot.subsystems.drive.Drive drive;
  private final java.util.function.DoubleSupplier xSupplier;
  private final java.util.function.DoubleSupplier ySupplier;
  private final Translation2d targetTranslation;

  private final PIDController headingController =
      new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);

  private Rotation2d heldHeading = new Rotation2d();
  private State state = State.ALIGNING;

  // --- Per-cycle updated values ---
  private Translation2d driverLinearVelocity = new Translation2d();
  private Translation2d shotOrigin = new Translation2d();
  private Translation2d fixedTarget = new Translation2d();
  private double distanceToTargetMeters = 0.0;
  private Rotation2d targetHeading = new Rotation2d();

  private double omegaPidRadPerSec = 0.0;
  private double omegaFfRadPerSec = 0.0;
  private double omegaRadPerSec = 0.0;

  private double hoodAngleDeg = 0.0;
  private double shooterSpeedRps = 0.0;
  private double shooterAccelFfRpsPerSec = 0.0;

  private boolean shootDistanceOk = false;
  private double flywheelMeasRps = 0.0;
  private double flywheelErrRps = 0.0;
  private double hoodErrDeg = 0.0;
  private double headingErrRad = 0.0;
  private boolean flywheelOk = false;
  private boolean hoodOk = false;
  private boolean headingOk = false;
  private boolean triggerHeld = false;

  private boolean readyToShoot = false;
  private boolean alignmentStillValid = false;
  private boolean flywheelDroppedForReaccel = false;
  private boolean flywheelRecoveredFromReaccel = false;

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
    addRequirements(drive, robotContainer.shooter, robotContainer.feeder, robotContainer.indexer);
  }

  @Override
  public void initialize() {
    headingController.reset();
    heldHeading = drive.getRotation();
    state = State.ALIGNING;
  }

  private void setState(State newState) {
    if (newState != state) {
      state = newState;
    }
  }

  private Translation2d getFixedTarget(
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

  private void stopFeed() {
    robotContainer.feeder.stop();
    robotContainer.indexer.stop();
  }

  private void runFeed() {
    robotContainer.feeder.setVelocity(FEEDER_RPS);
    robotContainer.indexer.setVoltage(INDEXER_VOLTS);
  }

  /** Returns shooter acceleration feedforward (RPS/s) from slope*dDot and updates logs. */
  private double getShooterAccelFfRpsPerSec() {
    double shooterSlope =
        slopeAtDistance(AutoShootConstants.shooterSpeedMap, distanceToTargetMeters, 0.15);
    ChassisSpeeds vField = drive.getFieldRelativeSpeeds();
    Translation2d aField = drive.getFieldRelativeAcceleration();
    Translation2d r =
        targetTranslation
            .minus(shotOrigin)
            .minus(
                new Translation2d(vField.vxMetersPerSecond, vField.vyMetersPerSecond)
                    .times(AutoShootConstants.FlyTime));
    double dDot = estimateDistanceRate(r, vField, aField, AutoShootConstants.FlyTime);
    double accelFf = shooterSlope * dDot * SHOOTER_ACCEL_FF_GAIN;

    Logger.recordOutput("AutoShoot/DistanceRate", dDot);
    Logger.recordOutput("AutoShoot/ShooterSlope", shooterSlope);
    Logger.recordOutput("AutoShoot/ShooterAccelFFRpsPerSec", accelFf);
    return accelFf;
  }

  private void aligning() {
    // Common per-cycle updates
    TraceTarget();
    // Shooter setpoints / FF
    hoodAngleDeg = AutoShootConstants.hoodAngleMap.get(distanceToTargetMeters);
    shooterSpeedRps = AutoShootConstants.shooterSpeedMap.get(distanceToTargetMeters);

    shooterAccelFfRpsPerSec = getShooterAccelFfRpsPerSec();

    Logger.recordOutput("AutoShoot/Distance", distanceToTargetMeters);
    Logger.recordOutput("AutoShoot/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("AutoShoot/ShooterSpeedRPS", shooterSpeedRps);
    updateBooleans();

    // Shooter control (aligning)
    robotContainer.shooter.setHoodAngle(hoodAngleDeg);
    robotContainer.shooter.setVelocity(shooterSpeedRps, shooterAccelFfRpsPerSec);

    stopFeed();
    if (readyToShoot) {
      setState(State.SHOOT);
    }
  }

  private void shoot() {
    // Common per-cycle updates
    TraceTarget();
    // Shooter setpoints / FF
    hoodAngleDeg = AutoShootConstants.hoodAngleMap.get(distanceToTargetMeters);
    shooterSpeedRps = AutoShootConstants.shooterSpeedMap.get(distanceToTargetMeters);

    shooterAccelFfRpsPerSec = getShooterAccelFfRpsPerSec();

    Logger.recordOutput("AutoShoot/Distance", distanceToTargetMeters);
    Logger.recordOutput("AutoShoot/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("AutoShoot/ShooterSpeedRPS", shooterSpeedRps);
    updateBooleans();

    // Shooter control (shooting)
    robotContainer.shooter.setHoodAngle(hoodAngleDeg);
    robotContainer.shooter.setVelocity(shooterSpeedRps, shooterAccelFfRpsPerSec);

    runFeed();

    if (!alignmentStillValid) {
      setState(State.ALIGNING);
      return;
    }
    if (flywheelDroppedForReaccel) {
      setState(State.REACCELERATE);
    }
  }

  private void reaccelerate() {
    // Common per-cycle updates
    TraceTarget();
    // Shooter setpoints / FF
    hoodAngleDeg = Math.max(AutoShootConstants.hoodAngleMap.get(distanceToTargetMeters) - 5, 0);
    shooterSpeedRps =
        Math.max(AutoShootConstants.shooterSpeedMap.get(distanceToTargetMeters) - 2, 0);

    shooterAccelFfRpsPerSec = getShooterAccelFfRpsPerSec();

    Logger.recordOutput("AutoShoot/Distance", distanceToTargetMeters);
    Logger.recordOutput("AutoShoot/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("AutoShoot/ShooterSpeedRPS", shooterSpeedRps);
    updateBooleans();

    // Shooter control (reaccelerate)
    robotContainer.shooter.setHoodAngle(hoodAngleDeg);
    robotContainer.shooter.setVelocity(shooterSpeedRps, REACCEL_BIG_ACCEL_FF_RPS_PER_SEC);

    runFeed();

    if (!alignmentStillValid) {
      setState(State.ALIGNING);
      return;
    }
    if (flywheelRecoveredFromReaccel) {
      setState(State.SHOOT);
    }
  }

  /** Common chain: compute target heading + omega and apply to drivetrain. */
  private void TraceTarget() {
    driverLinearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    shotOrigin = getShotOriginField();
    fixedTarget = getFixedTarget(shotOrigin, targetTranslation, drive.getFieldRelativeSpeeds());
    distanceToTargetMeters = fixedTarget.getNorm();

    boolean allowHeadingUpdate =
        distanceToTargetMeters >= HEADING_HOLD_MIN_DIST_METERS
            && distanceToTargetMeters <= HEADING_HOLD_MAX_DIST_METERS
            && distanceToTargetMeters > 1e-3;
    if (allowHeadingUpdate) {
      heldHeading = fixedTarget.getAngle();
    }
    targetHeading = heldHeading;

    Logger.recordOutput("AutoShoot/FixedTarget", fixedTarget.plus(shotOrigin));
    Logger.recordOutput("AutoShoot/ShotOrigin", shotOrigin);
    Logger.recordOutput("AutoShoot/HeadingAllowUpdate", allowHeadingUpdate);
    Logger.recordOutput("AutoShoot/HeadingHeldDeg", heldHeading.getDegrees());
    Logger.recordOutput(
        "AutoShoot/TargetPose", new Pose2d(drive.getPose().getTranslation(), targetHeading));

    omegaPidRadPerSec =
        headingController.calculate(drive.getRotation().getRadians(), targetHeading.getRadians());
    omegaFfRadPerSec = rotateOmegaTarget(drive.getFieldRelativeAcceleration());
    omegaRadPerSec =
        MathUtil.clamp(
            omegaPidRadPerSec + omegaFfRadPerSec,
            -drive.getMaxAngularSpeedRadPerSec(),
            drive.getMaxAngularSpeedRadPerSec());
    Logger.recordOutput("AutoShoot/HeadingOmegaPID", omegaPidRadPerSec);
    Logger.recordOutput("AutoShoot/HeadingOmegaFF", omegaFfRadPerSec);

    ChassisSpeeds fieldRelative =
        new ChassisSpeeds(
            driverLinearVelocity.getX() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            driverLinearVelocity.getY() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            omegaRadPerSec);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelative,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  /** Updates all boolean/threshold checks (does not command anything). */
  private void updateBooleans() {
    shootDistanceOk =
        distanceToTargetMeters >= MIN_SHOOT_DIST_METERS
            && distanceToTargetMeters <= MAX_SHOOT_DIST_METERS;
    flywheelMeasRps = robotContainer.shooter.getFlywheelVelocityRps();
    flywheelErrRps = shooterSpeedRps - flywheelMeasRps;
    hoodErrDeg = hoodAngleDeg - robotContainer.shooter.getHoodAngleDeg();
    headingErrRad =
        MathUtil.angleModulus(targetHeading.getRadians() - drive.getRotation().getRadians());

    flywheelOk = Math.abs(flywheelErrRps) <= FLYWHEEL_RPS_TOL;
    hoodOk = Math.abs(hoodErrDeg) <= HOOD_DEG_TOL;
    headingOk = Math.abs(headingErrRad) <= HEADING_TOL_RAD;
    triggerHeld = robotContainer.getRightTriggerAxisSupplier().getAsDouble() > 0.25;

    readyToShoot = shootDistanceOk && flywheelOk && hoodOk && headingOk && triggerHeld;
    alignmentStillValid = headingOk;

    flywheelDroppedForReaccel = flywheelMeasRps < (shooterSpeedRps - REACCEL_ENTER_DROP_RPS);
    flywheelRecoveredFromReaccel = flywheelMeasRps > (shooterSpeedRps - REACCEL_EXIT_DROP_RPS);

    Logger.recordOutput("AutoShoot/ShootDistanceOk", shootDistanceOk);
    Logger.recordOutput("AutoShoot/FlywheelMeasRps", flywheelMeasRps);
    Logger.recordOutput("AutoShoot/FlywheelErrRps", flywheelErrRps);
    Logger.recordOutput("AutoShoot/HoodErrDeg", hoodErrDeg);
    Logger.recordOutput("AutoShoot/HeadingErrDeg", Math.toDegrees(headingErrRad));
    Logger.recordOutput("AutoShoot/TriggerHeld", triggerHeld);
    Logger.recordOutput("AutoShoot/ReadyToShoot", readyToShoot);
    Logger.recordOutput("AutoShoot/AlignmentStillValid", alignmentStillValid);
    Logger.recordOutput("AutoShoot/FlywheelDropped", flywheelDroppedForReaccel);
    Logger.recordOutput("AutoShoot/FlywheelRecovered", flywheelRecoveredFromReaccel);
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
    Logger.recordOutput("AutoShoot/State", state.toString());
    switch (state) {
      case ALIGNING -> aligning();
      case SHOOT -> shoot();
      case REACCELERATE -> reaccelerate();
    }
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
    robotContainer.indexer.stop();
  }
}

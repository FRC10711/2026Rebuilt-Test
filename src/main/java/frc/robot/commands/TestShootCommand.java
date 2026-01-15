package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Test shooting command: spin up shooter + move hood to setpoint, and only run feeder when right
 * trigger is pressed.
 */
public class TestShootCommand extends Command {
  private final RobotContainer robotContainer;
  private final DoubleSupplier distanceMetersSupplier;
  private final double feederRps;
  private final double triggerThreshold;

  public TestShootCommand(
      RobotContainer robotContainer,
      DoubleSupplier distanceMetersSupplier,
      double feederRps,
      double triggerThreshold) {
    this.robotContainer = robotContainer;
    this.distanceMetersSupplier = distanceMetersSupplier;
    this.feederRps = feederRps;
    this.triggerThreshold = triggerThreshold;
    addRequirements(robotContainer.shooter, robotContainer.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Distance input is assumed to be from robot center; convert to distance from shooter exit.
    double distanceFromCenter = distanceMetersSupplier.getAsDouble();
    double distanceFromShooter =
        Math.max(0.0, distanceFromCenter - Constants.shooterConstants.FLYWHEEL_OFFSET_X_METERS);

    double shooterRps = AutoShootConstants.shooterSpeedMap.get(distanceFromShooter);
    double hoodDeg = AutoShootConstants.hoodAngleMap.get(distanceFromShooter);

    Logger.recordOutput("TestShoot/DistanceFromCenterMeters", distanceFromCenter);
    Logger.recordOutput("TestShoot/DistanceFromShooterMeters", distanceFromShooter);
    Logger.recordOutput("TestShoot/ShooterRpsSetpoint", shooterRps);
    Logger.recordOutput("TestShoot/HoodDegSetpoint", hoodDeg);

    robotContainer.shooter.setVelocity(shooterRps);
    robotContainer.shooter.setHoodAngle(hoodDeg);

    if (robotContainer.getRightTriggerAxisSupplier().getAsDouble() > triggerThreshold) {
      robotContainer.feeder.setVelocity(feederRps);
    } else {
      robotContainer.feeder.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    robotContainer.feeder.stop();
    robotContainer.shooter.stop();
  }
}

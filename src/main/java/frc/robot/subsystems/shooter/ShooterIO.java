// Copyright 2025
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean connected = false;

    // Flywheel (mechanism units after SensorToMechanismRatio config)
    public double flywheelLeaderVelocityRadPerSec = 0.0;
    public double flywheelFollowerVelocityRadPerSec = 0.0;
    public double flywheelLeaderAppliedVolts = 0.0;
    public double flywheelFollowerAppliedVolts = 0.0;
    public double flywheelLeaderCurrentAmps = 0.0;
    public double flywheelFollowerCurrentAmps = 0.0;

    // Hood/backplate (mechanism units after SensorToMechanismRatio config)
    public double hoodAngleDeg = 0.0;
    public double hoodVelocityDegPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets flywheel speed in RPM (mechanism RPM). */
  public default void setFlywheelVelocity(double rpm) {}

  /** Sets hood/backplate angle in degrees (mechanism degrees). */
  public default void setHoodAngleDeg(double deg) {}

  public default void stop() {}
}

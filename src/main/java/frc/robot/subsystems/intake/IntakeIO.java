// Copyright 2025
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;

    // Mechanism units after SensorToMechanismRatio config
    public double leaderVelocityRadPerSec = 0.0;
    public double followerVelocityRadPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double followerAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double followerCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets intake voltage output (volts). */
  public default void setVoltage(double volts) {}

  public default void stop() {}
}

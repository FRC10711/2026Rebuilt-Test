// Copyright 2025
package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FeederIO {
  public static class FeederIOInputs implements LoggableInputs {
    public boolean connected = false;

    // Mechanism units after SensorToMechanismRatio config
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("VelocityRadPerSec", velocityRadPerSec);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      velocityRadPerSec = table.get("VelocityRadPerSec", velocityRadPerSec);
      appliedVolts = table.get("AppliedVolts", appliedVolts);
      currentAmps = table.get("CurrentAmps", currentAmps);
    }
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  /** Sets feeder velocity (mechanism rotations per second after SensorToMechanismRatio). */
  public default void setVelocity(double rps) {}

  public default void stop() {}
}

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IndexerIO {
  public static class IndexerIOInputs implements LoggableInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      appliedVolts = table.get("AppliedVolts", appliedVolts);
      currentAmps = table.get("CurrentAmps", currentAmps);
    }
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Sets indexer output voltage (volts). */
  public default void setVoltage(double volts) {}

  public default void stop() {}
}

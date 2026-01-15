package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private double voltageSetpoint = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/VoltageSetpoint", voltageSetpoint);
  }

  /** Sets intake output voltage (volts). */
  public void setVoltage(double volts) {
    voltageSetpoint = volts;
    io.setVoltage(volts);
  }

  public void stop() {
    voltageSetpoint = 0.0;
    io.stop();
  }
}

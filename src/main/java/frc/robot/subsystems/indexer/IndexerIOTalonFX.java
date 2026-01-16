package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX motor;

  private final VoltageOut voltageReq = new VoltageOut(0.0);

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  public IndexerIOTalonFX() {
    motor = new TalonFX(Constants.indexerConstants.MOTOR_ID);

    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted =
        Constants.indexerConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(cfg);

    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVolts, current);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(appliedVolts, current);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.connected = status.isOK();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageReq.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}

// Copyright 2025
package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX motor;
  private final TalonFX followerMotor;

  // Control request
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentReq =
      new VelocityTorqueCurrentFOC(0.0);
  private final Follower followerReq;

  // Signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  public FeederIOTalonFX() {
    motor = new TalonFX(Constants.feederConstants.MOTOR_ID);
    followerMotor = new TalonFX(Constants.feederConstants.FOLLOWER_MOTOR_ID);

    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted =
        Constants.feederConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    cfg.Feedback.SensorToMechanismRatio = Constants.feederConstants.SENSOR_TO_MECH_RATIO;
    cfg.Slot0 =
        new Slot0Configs()
            .withKP(Constants.feederConstants.KP)
            .withKI(Constants.feederConstants.KI)
            .withKD(Constants.feederConstants.KD)
            .withKV(Constants.feederConstants.KV)
            .withKS(Constants.feederConstants.KS);
    // Optional: keep reverse torque at 0 so the feeder doesn't "suck back" when commanded near
    // zero.
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = false;
    cfg.CurrentLimits.StatorCurrentLimitEnable = false;
    motor.getConfigurator().apply(cfg);

    var followerCfg = new TalonFXConfiguration();
    followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerCfg.Feedback.SensorToMechanismRatio = Constants.feederConstants.SENSOR_TO_MECH_RATIO;
    followerCfg.Slot0 = cfg.Slot0;
    followerCfg.CurrentLimits.SupplyCurrentLimitEnable = false;
    followerCfg.CurrentLimits.StatorCurrentLimitEnable = false;
    followerMotor.getConfigurator().apply(followerCfg);

    followerReq = new Follower(motor.getDeviceID(), Constants.feederConstants.FOLLOWER_ALIGNMENT);
    followerMotor.setControl(followerReq);

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current);
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.connected = status.isOK();
  }

  @Override
  public void setVelocity(double rps) {
    // Phoenix expects mechanism rotations per second after SensorToMechanismRatio
    motor.setControl(velocityTorqueCurrentReq.withVelocity(rps));
  }

  @Override
  public void stop() {
    motor.stopMotor();
    followerMotor.stopMotor();
  }
}

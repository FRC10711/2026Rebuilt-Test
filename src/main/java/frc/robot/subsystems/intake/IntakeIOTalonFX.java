// Copyright 2025
package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX leader;
  private final TalonFX follower;

  // Voltage control requests
  private final VoltageOut leaderVoltageReq = new VoltageOut(0.0);
  private final VoltageOut followerVoltageReq = new VoltageOut(0.0);

  // Status signals
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Current> followerCurrent;

  public IntakeIOTalonFX() {
    leader = new TalonFX(Constants.intakeConstants.LEADER_MOTOR_ID);
    follower = new TalonFX(Constants.intakeConstants.FOLLOWER_MOTOR_ID);

    var leaderCfg = new TalonFXConfiguration();
    leaderCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderCfg.MotorOutput.Inverted =
        Constants.intakeConstants.LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leader.getConfigurator().apply(leaderCfg);

    var followerCfg = new TalonFXConfiguration();
    followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerCfg.MotorOutput.Inverted =
        Constants.intakeConstants.FOLLOWER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    follower.getConfigurator().apply(followerCfg);

    // Acquire status signals
    leaderVelocity = leader.getVelocity();
    followerVelocity = follower.getVelocity();
    leaderAppliedVolts = leader.getMotorVoltage();
    followerAppliedVolts = follower.getMotorVoltage();
    leaderCurrent = leader.getStatorCurrent();
    followerCurrent = follower.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderVelocity,
        followerVelocity,
        leaderAppliedVolts,
        followerAppliedVolts,
        leaderCurrent,
        followerCurrent);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            leaderVelocity,
            followerVelocity,
            leaderAppliedVolts,
            followerAppliedVolts,
            leaderCurrent,
            followerCurrent);

    inputs.leaderVelocityRadPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.followerVelocityRadPerSec =
        Units.rotationsToRadians(followerVelocity.getValueAsDouble());
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();

    inputs.connected = status.isOK();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(leaderVoltageReq.withOutput(volts));
    follower.setControl(followerVoltageReq.withOutput(volts));
  }

  @Override
  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }
}

// Copyright 2025
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX flywheelLeader;
  private final TalonFX flywheelFollower;
  private final TalonFX hoodMotor;

  // Control requests
  private final VelocityTorqueCurrentFOC flywheelVelocityReq = new VelocityTorqueCurrentFOC(0.0);
  private final MotionMagicTorqueCurrentFOC hoodMotionMagicReq =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final Follower flywheelFollowerReq;

  // Flywheel signals
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Current> followerCurrent;

  // Hood signals
  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodCurrent;

  public ShooterIOTalonFX() {
    flywheelLeader = new TalonFX(Constants.shooterConstants.FLYWHEEL_LEADER_ID);
    flywheelFollower = new TalonFX(Constants.shooterConstants.FLYWHEEL_FOLLOWER_ID);
    hoodMotor = new TalonFX(Constants.shooterConstants.HOOD_MOTOR_ID);

    // Configure flywheel motors (velocity closed-loop in Slot0, sensor ratio to mechanism)
    var flywheelCfg = new TalonFXConfiguration();
    flywheelCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelCfg.MotorOutput.Inverted =
        Constants.shooterConstants.FLYWHEEL_LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    flywheelCfg.TorqueCurrent.PeakReverseTorqueCurrent = 0;
    flywheelCfg.Feedback.SensorToMechanismRatio =
        Constants.shooterConstants.FLYWHEEL_SENSOR_TO_MECH_RATIO;
    flywheelCfg.Slot0 =
        new Slot0Configs()
            .withKP(Constants.shooterConstants.FLYWHEEL_KP)
            .withKI(Constants.shooterConstants.FLYWHEEL_KI)
            .withKD(Constants.shooterConstants.FLYWHEEL_KD)
            .withKV(Constants.shooterConstants.FLYWHEEL_KV)
            .withKS(Constants.shooterConstants.FLYWHEEL_KS);
    flywheelLeader.getConfigurator().apply(flywheelCfg);

    var followerCfg = new TalonFXConfiguration();
    followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerCfg.Feedback.SensorToMechanismRatio =
        Constants.shooterConstants.FLYWHEEL_SENSOR_TO_MECH_RATIO;
    followerCfg.Slot0 = flywheelCfg.Slot0;
    flywheelFollower.getConfigurator().apply(followerCfg);

    flywheelFollowerReq =
        new Follower(
            flywheelLeader.getDeviceID(), Constants.shooterConstants.FLYWHEEL_FOLLOWER_INVERTED);
    flywheelFollower.setControl(flywheelFollowerReq);

    // Configure hood motor for Motion Magic Voltage (position closed-loop in Slot0)
    var hoodCfg = new TalonFXConfiguration();
    hoodCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodCfg.MotorOutput.Inverted =
        Constants.shooterConstants.HOOD_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    hoodCfg.Feedback.SensorToMechanismRatio = Constants.shooterConstants.HOOD_SENSOR_TO_MECH_RATIO;
    hoodCfg.Slot0 =
        new Slot0Configs()
            .withKP(Constants.shooterConstants.HOOD_KP)
            .withKI(Constants.shooterConstants.HOOD_KI)
            .withKD(Constants.shooterConstants.HOOD_KD)
            .withKS(Constants.shooterConstants.HOOD_KS)
            .withKV(Constants.shooterConstants.HOOD_KV)
            .withKA(Constants.shooterConstants.HOOD_KA)
            .withKG(Constants.shooterConstants.HOOD_KG);
    hoodCfg.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.shooterConstants.HOOD_MM_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.shooterConstants.HOOD_MM_ACCELERATION)
            .withMotionMagicJerk(Constants.shooterConstants.HOOD_MM_JERK);
    hoodMotor.getConfigurator().apply(hoodCfg);
    hoodMotor.setPosition(0);
    // Acquire status signals
    leaderVelocity = flywheelLeader.getVelocity();
    followerVelocity = flywheelFollower.getVelocity();
    leaderAppliedVolts = flywheelLeader.getMotorVoltage();
    followerAppliedVolts = flywheelFollower.getMotorVoltage();
    leaderCurrent = flywheelLeader.getStatorCurrent();
    followerCurrent = flywheelFollower.getStatorCurrent();

    hoodPosition = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();
    hoodAppliedVolts = hoodMotor.getMotorVoltage();
    hoodCurrent = hoodMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderVelocity,
        followerVelocity,
        leaderAppliedVolts,
        followerAppliedVolts,
        leaderCurrent,
        followerCurrent,
        hoodPosition,
        hoodVelocity,
        hoodAppliedVolts,
        hoodCurrent);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var flywheelStatus =
        BaseStatusSignal.refreshAll(
            leaderVelocity,
            followerVelocity,
            leaderAppliedVolts,
            followerAppliedVolts,
            leaderCurrent,
            followerCurrent);
    var hoodStatus =
        BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodAppliedVolts, hoodCurrent);

    inputs.flywheelLeaderVelocityRotationPerSec = leaderVelocity.getValueAsDouble();
    inputs.flywheelFollowerVelocityRotationPerSec = followerVelocity.getValueAsDouble();
    inputs.flywheelLeaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.flywheelFollowerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.flywheelLeaderCurrentAmps = leaderCurrent.getValueAsDouble();
    inputs.flywheelFollowerCurrentAmps = followerCurrent.getValueAsDouble();

    inputs.hoodAngleDeg = hoodPosition.getValueAsDouble() * 360.0;
    inputs.hoodVelocityDegPerSec = hoodVelocity.getValueAsDouble() * 360.0;
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();

    inputs.connected = flywheelStatus.isOK() && hoodStatus.isOK();
  }

  @Override
  public void setFlywheelVelocity(double rps) {
    // Phoenix expects mechanism rotations per second after SensorToMechanismRatio
    flywheelLeader.setControl(flywheelVelocityReq.withVelocity(rps));
    // Keep follower following leader (in case another control mode was applied elsewhere)
    flywheelFollower.setControl(flywheelFollowerReq);
  }

  @Override
  public void setFlywheelVelocity(double rps, double accelRpsPerSec) {
    // Phoenix expects mechanism rotations per second after SensorToMechanismRatio
    flywheelLeader.setControl(
        flywheelVelocityReq.withVelocity(rps).withAcceleration(accelRpsPerSec));
    flywheelFollower.setControl(flywheelFollowerReq);
  }

  @Override
  public void setHoodAngleDeg(double deg) {
    // Phoenix expects mechanism rotations after SensorToMechanismRatio
    double rotations = deg / 360.0;
    hoodMotor.setControl(hoodMotionMagicReq.withPosition(rotations));
  }

  @Override
  public void setHoodAngleDeg(double deg, double velDegPerSec) {
    // Phoenix expects mechanism rotations/rotations per second after SensorToMechanismRatio
    double rotations = deg / 360.0;
    // MotionMagic request type doesn't expose a velocity setpoint; use Slot0 KV/KA/KS for FF
    // and optionally tune MotionMagic cruise/accel/jerk.
    // (We still log/use velDegPerSec upstream for analysis.)
    hoodMotor.setControl(hoodMotionMagicReq.withPosition(rotations));
  }

  @Override
  public void stop() {
    flywheelLeader.stopMotor();
    flywheelFollower.stopMotor();
    hoodMotor.stopMotor();
  }
}

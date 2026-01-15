// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Intake CAN IDs and tuning. Update these to match your robot wiring. */
  public static final class intakeConstants {
    // CAN IDs
    public static final int LEADER_MOTOR_ID = 50;
    public static final int FOLLOWER_MOTOR_ID = 19;

    // Motor directions
    public static final boolean LEADER_INVERTED = false;
    public static final boolean FOLLOWER_INVERTED = true;

    private intakeConstants() {}
  }

  public static final class fieldConstants {
    public static final double HUB_HEIGHT_METERS = 2.64;
    public static final Translation2d RED_HUB_LOCATION = new Translation2d(11.917, 4.030);
    public static final Translation2d BLUE_HUB_LOCATION = new Translation2d(4.623, 4.030);

    public static final Translation2d getHubLocation(Alliance alliance) {

      return alliance == Alliance.Red ? RED_HUB_LOCATION : BLUE_HUB_LOCATION;
    }
  }

  public static final class AutoShootConstants {
    public static final double FlyTime = 0.9;
    public static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
    public static final double MAX_SHOOTING_VELOCITY = 3;

    static {
      // hoodAngleMap.put(0.0, 0.0);
      // hoodAngleMap.put(0.506, 5.0);
      // hoodAngleMap.put(1.02, 10.0);
      // hoodAngleMap.put(1.550, 15.0);
      // hoodAngleMap.put(2.106, 20.0);
      // hoodAngleMap.put(2.70, 25.);
      // hoodAngleMap.put(3.34, 30.0);
      // hoodAngleMap.put(4.05, 35.0);
      // hoodAngleMap.put(4.85, 40.0);
      // hoodAngleMap.put(5.787, 45.);
      hoodAngleMap.put(1.65, 1.);
      hoodAngleMap.put(2.18, 5.);
      hoodAngleMap.put(2.76, 11.);
      hoodAngleMap.put(3.42, 16.);
      hoodAngleMap.put(4.07, 18.);
      shooterSpeedMap.put(1.65, 29.7);
      shooterSpeedMap.put(2.18, 32.);
      shooterSpeedMap.put(2.76, 34.);
      shooterSpeedMap.put(3.42, 35.5);
      hoodAngleMap.put(4.07, 37.);

      // shooterSpeedMap.put(0.0, 54.48);
      // shooterSpeedMap.put(0.506, 54.69);
      // shooterSpeedMap.put(1.02, 55.32);
      // shooterSpeedMap.put(1.550, 56.40);
      // shooterSpeedMap.put(2.106, 57.98);
      // shooterSpeedMap.put(2.70, 60.117);
      // shooterSpeedMap.put(3.34, 62.913);
      // shooterSpeedMap.put(4.05, 66.513);
      // shooterSpeedMap.put(4.85, 71.125);
      // shooterSpeedMap.put(5.787, 77.053);
    }
  }
  /** Shooter CAN IDs and tuning. Update these to match your robot wiring & tuning. */
  public static final class shooterConstants {
    // CAN IDs
    public static final int FLYWHEEL_LEADER_ID = 15;
    public static final int FLYWHEEL_FOLLOWER_ID = 16;
    public static final int HOOD_MOTOR_ID = 14;

    // Motor directions
    public static final boolean FLYWHEEL_LEADER_INVERTED = false;

    public static final MotorAlignmentValue FLYWHEEL_FOLLOWER_INVERTED =
        MotorAlignmentValue.Opposed;
    public static final boolean HOOD_INVERTED = true;

    // Gear ratios (motor rotations per mechanism rotation)
    public static final double FLYWHEEL_SENSOR_TO_MECH_RATIO = 1.0;
    public static final double HOOD_SENSOR_TO_MECH_RATIO = 31.875;

    /** Flywheel/exit location relative to robot center (meters). +X forward, +Y left. */
    public static final double FLYWHEEL_OFFSET_X_METERS = 0.18;

    public static final double FLYWHEEL_OFFSET_Y_METERS = 0.0;

    // Flywheel closed-loop gains (Phoenix Slot0)
    public static final double FLYWHEEL_KP = 7;
    public static final double FLYWHEEL_KI = 0.0;
    public static final double FLYWHEEL_KD = 0.0;
    public static final double FLYWHEEL_KV = 0.0;
    public static final double FLYWHEEL_KS = 3.5;

    // Hood Motion Magic (mechanism rotations/sec and rotations/sec^2)
    public static final double HOOD_MM_CRUISE_VELOCITY = 0.5;
    public static final double HOOD_MM_ACCELERATION = 2.0;
    public static final double HOOD_MM_JERK = 0.0;

    // Hood closed-loop gains (Phoenix Slot0)
    public static final double HOOD_KP = 2300;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KD = 230.0;
    public static final double HOOD_KS = 0.0;
    public static final double HOOD_KG = 2.0;
    public static final double HOOD_KV = 0.0;
    public static final double HOOD_KA = 0.0;

    private shooterConstants() {}
  }

  /**
   * Drivetrain torque-current deadband tuning (applies when
   * DriveMotorClosedLoopOutput=TorqueCurrentFOC).
   */
  public static final class drivetrainConstants {
    /**
     * Deadband for torque-current requests (same units as {@code TorqueCurrentFOC.withOutput},
     * amps).
     */
    public static final double TORQUE_CURRENT_DEADBAND_AMPS = 0.0;

    private drivetrainConstants() {}
  }

  /** Feeder CAN IDs and tuning. Update these to match your robot wiring & tuning. */
  public static final class feederConstants {
    // CAN IDs
    public static final int MOTOR_ID = 21;

    // Motor direction
    public static final boolean INVERTED = false;

    // Gear ratio (motor rotations per mechanism rotation)
    public static final double SENSOR_TO_MECH_RATIO = 1.0;

    // Velocity closed-loop gains (Phoenix Slot0)
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KV = 0.;

    private feederConstants() {}
  }
}

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

  /** Vision tuning for Limelight MegaTag2 pose updates. */
  public static final class visionConstants {
    /** TA->XY std dev map (meters). Tune these points based on your camera mounting & lighting. */
    public static final InterpolatingDoubleTreeMap taToXYStdDevMeters =
        new InterpolatingDoubleTreeMap();

    /**
     * Theta std dev (radians). Use a huge value to effectively ignore vision heading correction.
     * (We also override the vision measurement rotation to the current gyro heading.)
     */
    public static final double thetaStdDevRad = 1.0e6;

    static {
      // Placeholder tuning points (TA is % of image, 0-100). Adjust to your robot.
      // Larger TA (bigger tag) => lower std dev (more trust)
      taToXYStdDevMeters.put(0.17, 0.08);
      taToXYStdDevMeters.put(0.22, 0.20);
      taToXYStdDevMeters.put(0.071, 0.35);
      taToXYStdDevMeters.put(0.046, 0.4);
      taToXYStdDevMeters.put(0.03, 0.7);
      taToXYStdDevMeters.put(0.01, 1.);
    }

    private visionConstants() {}
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
      hoodAngleMap.put(2.02, 3.2);
      hoodAngleMap.put(2.65, 6.);
      hoodAngleMap.put(3.42, 9.);
      hoodAngleMap.put(4.08, 18.);
      hoodAngleMap.put(5.05, 18.);
      shooterSpeedMap.put(2.02, 33.5);
      shooterSpeedMap.put(2.65, 35.);
      shooterSpeedMap.put(3.42, 37.);
      shooterSpeedMap.put(4.08, 37.);
      shooterSpeedMap.put(5.05, 45.8);

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
    public static final double FLYWHEEL_OFFSET_X_METERS = 0.1;

    public static final double FLYWHEEL_OFFSET_Y_METERS = 0.0;

    // Flywheel closed-loop gains (Phoenix Slot0)
    public static final double FLYWHEEL_KP = 6;
    public static final double FLYWHEEL_KI = 0.0;
    public static final double FLYWHEEL_KD = 0.0;
    public static final double FLYWHEEL_KV = 0.0;
    public static final double FLYWHEEL_KS = 3.5;

    // Hood Motion Magic (mechanism rotations/sec and rotations/sec^2)
    public static final double HOOD_MM_CRUISE_VELOCITY = 0.5;
    public static final double HOOD_MM_ACCELERATION = 1.0;
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
    public static final int FOLLOWER_MOTOR_ID = 34;

    // Motor direction
    public static final boolean INVERTED = false;
    /** Feeder follower alignment relative to leader. Use Opposed to run opposite direction. */
    public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;

    // Gear ratio (motor rotations per mechanism rotation)
    public static final double SENSOR_TO_MECH_RATIO = 1.0;

    // Velocity closed-loop gains (Phoenix Slot0)
    public static final double KP = 10.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KV = 0.;
    public static final double KS = 5;

    private feederConstants() {}
  }

  /** Indexer CAN IDs and tuning. Update these to match your robot wiring & tuning. */
  public static final class indexerConstants {
    public static final int MOTOR_ID = 35;
    public static final boolean INVERTED = false;

    private indexerConstants() {}
  }
}

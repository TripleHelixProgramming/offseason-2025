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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  // TODO: Is this necessary?
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

  public final class RobotConstants {
    public static final double kNominalVoltage = 12.0;
  }

  public enum MotorConstants {
    NEO(RPM.of(5676), 60),
    NEO_550(RPM.of(11000), 20),
    NEO_VORTEX(RPM.of(6784), 60),
    KRAKEN_X60(RPM.of(6000), 60);

    public final AngularVelocity freeSpeed;
    public final int defaultSupplyCurrentLimit;

    MotorConstants(AngularVelocity freeSpeed, int supplyCurrentLimit) {
      this.freeSpeed = freeSpeed;
      this.defaultSupplyCurrentLimit = supplyCurrentLimit;
    }
  }

  public static final class AutoConstants {
    public static final int kAllianceColorSelectorPort = 10;

    // max length is 8
    public static final int[] kAutonomousModeSelectorPorts = {11, 12, 13, 18, 19};
  }

  public static final class OIConstants {
    public static final int kDefaultDriverPort = 0;
    public static final int kDefaultOperatorPort = 1;
  }
}

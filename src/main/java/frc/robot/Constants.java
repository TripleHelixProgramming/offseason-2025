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

  public static final class MotorConstants {
    public static final class NEOConstants {
      public static final AngularVelocity kFreeSpeed = RPM.of(5676);
      public static final int kDefaultCurrentLimit = 60;
    }

    public static final class NEO550Constants {
      public static final AngularVelocity kFreeSpeed = RPM.of(11000);
      public static final int kDefaultCurrentLimit = 20;
    }

    public static final class NEOVortexConstants {
      public static final AngularVelocity kFreeSpeed = RPM.of(6784);
      public static final int kDefaultCurrentLimit = 60;
    }
  }

  public static final class AutoConstants {
    public static final int kAllianceColorSelectorPort = 10;

    // max length is 8
    public static final int[] kAutonomousModeSelectorPorts = {11, 12, 13, 18, 19};
  }

  public static final class OIConstants {
    public static final int kDefaultDriverControllerPort = 0;
    public static final int kDefaultOperatorControllerPort = 1;
  }
}

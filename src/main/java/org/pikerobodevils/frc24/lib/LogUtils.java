/* Copyright 2023 Pike RoboDevils, FRC Team 1018
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE.md file or
 * at https://opensource.org/licenses/MIT. */

package org.pikerobodevils.frc24.lib;

import static edu.wpi.first.wpilibj.DataLogManager.log;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.util.WPILibVersion;
// import org.tinylog.Logger;

public class LogUtils {
  private static StringLogEntry m_messageLog;
  public static final String METADATA_PATH = "/RealMetadata/";

  private static void init() {
    if (m_messageLog == null) {
      m_messageLog = new StringLogEntry(DataLogManager.getLog(), "messages");
    }
  }

  public static void logMetadataEntry(String key, Object value) {
    var entry = new StringLogEntry(DataLogManager.getLog(), METADATA_PATH + key);
    entry.append(value.toString());
    entry.finish();
    // Logger.tag("Metadata").info(String.format("%-25s%s", key + ":", value));
  }

  public static void logManifestMetadata(Object object) {
    log("Build debug info:");
    Util.getManifestAttributesForClass(object)
        .forEach(
            (key, value) -> {
              LogUtils.logMetadataEntry(key.toString(), value.toString());
            });
  }

  public static void logSoftwareVersionMetadata() {
    log("Software Versions:");
    logMetadataEntry(
        "Java Version",
        System.getProperty("java.vendor") + " " + System.getProperty("java.version"));
    logMetadataEntry("WPILib Version", WPILibVersion.Version);
    logMetadataEntry("RevLib Version", CANSparkMax.kAPIVersion);
  }

  public static void logNoPrint(String message) {
    init();
    m_messageLog.append(message);
  }

  // @Deprecated
  // public static void warning(String warning) {
  //   Logger.tag("LogUtils").warn(warning);
  // }
}

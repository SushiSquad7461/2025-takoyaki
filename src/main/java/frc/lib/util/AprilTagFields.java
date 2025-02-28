package frc.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

/** Loadable AprilTag field layouts. */
public enum AprilTagFields {
  /** 2025 Reefscape. */
  k2025Reefscape("2025-reefscape-welded.json"),
  /** 2025 Reefscape Andymark. */
  k2025ReefscapeAndymark("2025-reefscape-andymark.json");

  /** Base resource directory. */
  public static final String kBaseResourceDir = "/frc/lib/apriltag/";

  /** Alias to the current game. */
  public static final AprilTagFields kDefaultField = k2025Reefscape;

  /** Resource filename. */
  public final String m_resourceFile;

  AprilTagFieldLayout m_fieldLayout;

  AprilTagFields(String resourceFile) {
    m_resourceFile = kBaseResourceDir + resourceFile;
  }
}

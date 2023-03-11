package frc.robot.commands.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum PreDefPose {
    BlueLeftCorner("BlueLeftCorner", new Pose2d(1.88, 5.08, new Rotation2d())),
    BlueLineRight("BlueLineRight", new Pose2d(1.88, 3.58, new Rotation2d())),
    BlueRightCorner("BlueRightCorner", new Pose2d(1.88, 0.41, new Rotation2d())),

    RedLeftCorner("RedLeftCorner", new Pose2d(14.66, 5.08, new Rotation2d(-1, 0))),
    RedLineRight("RedLineRight", new Pose2d(14.66, 3.58, new Rotation2d(-1, 0))),
    RedRightCorner("RedRightCorner", new Pose2d(14.66, 0.41, new Rotation2d(-1, 0))),

    BluePiece1("BluePiece1", new Pose2d(1.22, 7.07, new Rotation2d())),
    BluePiece2("BluePiece2", new Pose2d(3.36, 7.07, new Rotation2d())),
    BluePiece3("BluePiece3", new Pose2d(2.14, 7.07, new Rotation2d())),
    BluePiece4("BluePiece4", new Pose2d(0.92, 7.07, new Rotation2d())),

    RedPiece1("RedPiece1", new Pose2d(1.22, 9.97, new Rotation2d(-1, 0))),
    RedPiece2("RedPiece2", new Pose2d(3.36, 9.97, new Rotation2d(-1, 0))),
    RedPiece3("RedPiece3", new Pose2d(2.14, 9.97, new Rotation2d(-1, 0))),
    RedPiece4("RedPiece4", new Pose2d(0.92, 9.97, new Rotation2d(-1, 0))),

    BlueChStInside("BlueChStIn", new Pose2d(5.5, 2.73, new Rotation2d(-1, 0))),
    RedChStInside("RedChStIn", new Pose2d(11, 2.73, new Rotation2d())),
    ;

    private final String m_nameInPath;
    private final Pose2d m_pose2d;

    private PreDefPose(String nameInPath, Pose2d pose2d) {
        m_nameInPath = nameInPath;
        m_pose2d = pose2d;
    }

    public String toString() {
        return m_nameInPath;
    }

    public Pose2d toPose() {
        return m_pose2d;
    }
}

package frc.robot.commands.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PreDefPose {

    public enum PoseOnField {
        BlueLeftCorner, BlueRightCorner, BlueLineRight,
        RedLeftCorner, RedRightCorner, RedLineLeft,

        BluePiece1, BluePiece2, BluePiece3, BluePiece4,
        RedPiece1, RedPiece2, RedPiece3, RedPiece4;
    }
    
    public static class PosesTranslation {
        private static PosesTranslation m_instance;
        public static PosesTranslation getInstance() { return m_instance == null? new PosesTranslation(): m_instance; }

        HashMap<PoseOnField, Pose2d> m_TranslationToPose = new HashMap<PoseOnField, Pose2d>();
        HashMap<PoseOnField, String> m_TranslationToPathName = new HashMap<PoseOnField, String>();

        private PosesTranslation() {
            m_TranslationToPose.put(PoseOnField.BlueLeftCorner, new Pose2d(1.88, 5.08, new Rotation2d()));
            m_TranslationToPose.put(PoseOnField.BlueLineRight, new Pose2d(1.88, 3.58, new Rotation2d()));
            m_TranslationToPose.put(PoseOnField.BlueRightCorner, new Pose2d(1.88, 0.41, new Rotation2d()));
            m_TranslationToPose.put(PoseOnField.RedLeftCorner, new Pose2d(14.66, 5.08, new Rotation2d(-1, 0)));
            m_TranslationToPose.put(PoseOnField.RedLineLeft, new Pose2d(14.66, 3.58, new Rotation2d(-1, 0)));
            m_TranslationToPose.put(PoseOnField.RedRightCorner, new Pose2d(14.66, 0.41, new Rotation2d(-1, 0)));

            m_TranslationToPose.put(PoseOnField.BluePiece1, new Pose2d(1.22, 7.07, new Rotation2d()));
            m_TranslationToPose.put(PoseOnField.BluePiece2, new Pose2d(3.36, 7.07, new Rotation2d()));
            m_TranslationToPose.put(PoseOnField.BluePiece3, new Pose2d(2.14, 7.07, new Rotation2d()));
            m_TranslationToPose.put(PoseOnField.BluePiece4, new Pose2d(0.92, 7.07, new Rotation2d()));

            m_TranslationToPose.put(PoseOnField.RedPiece1, new Pose2d(1.22, 9.97, new Rotation2d(-1, 0)));
            m_TranslationToPose.put(PoseOnField.RedPiece2, new Pose2d(3.36, 9.97, new Rotation2d(-1, 0)));
            m_TranslationToPose.put(PoseOnField.RedPiece3, new Pose2d(2.14, 9.97, new Rotation2d(-1, 0)));
            m_TranslationToPose.put(PoseOnField.RedPiece4, new Pose2d(0.92, 9.97, new Rotation2d(-1, 0)));

            m_TranslationToPathName.put(PoseOnField.BlueLeftCorner, "BlueLeftCorner");
            m_TranslationToPathName.put(PoseOnField.BlueLineRight, "BlueLineRIght");
            m_TranslationToPathName.put(PoseOnField.BlueRightCorner, "BlueRightCorner");
            m_TranslationToPathName.put(PoseOnField.RedLeftCorner, "RedLeftCorner");
            m_TranslationToPathName.put(PoseOnField.RedLineLeft, "RedLineLeft");
            m_TranslationToPathName.put(PoseOnField.RedRightCorner, "RedRightCorner");

            m_TranslationToPathName.put(PoseOnField.BluePiece1, "BluePiece1");
            m_TranslationToPathName.put(PoseOnField.BluePiece2, "BluePiece2");
            m_TranslationToPathName.put(PoseOnField.BluePiece3, "BluePiece3");
            m_TranslationToPathName.put(PoseOnField.BluePiece4, "BluePiece4");

            m_TranslationToPathName.put(PoseOnField.RedPiece1, "RedPiece1");
            m_TranslationToPathName.put(PoseOnField.RedPiece2, "RedPiece2");
            m_TranslationToPathName.put(PoseOnField.RedPiece3, "RedPiece3");
            m_TranslationToPathName.put(PoseOnField.RedPiece4, "RedPiece4");

        }

        public Pose2d getPose(PoseOnField poseOnFieled) {
            return m_TranslationToPose.get(poseOnFieled);
        }

        public String getPathName(PoseOnField poseOnField) {
            return m_TranslationToPathName.get(poseOnField);
        }
    }

    private PoseOnField m_poseOnField;
    public HashMap<PoseOnField, Pose2d> poses = new HashMap<PoseOnField, Pose2d>();

    public PreDefPose(PoseOnField poseOnField) {
        m_poseOnField = poseOnField;
    }

    public Pose2d toPose() {
        return PosesTranslation.getInstance().getPose(m_poseOnField);
    }

    public String toString() {
        return PosesTranslation.getInstance().getPathName(m_poseOnField);
    }
}


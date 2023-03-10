package frc.robot;

public class Utils {
    public static enum StartingPosition {
        LEFT, MID, RIGHT
    }

    // public static class Pieces {
    public enum PiecePosition {
        ONE,
        TWO,
        THREE,
        FOUR;

        public String toString(PiecePosition pos) {
            switch (pos) {
                case ONE: return "Piece1";
                case TWO: return "Piece2";
                case THREE: return "Piece3";
                case FOUR: return "Piece4";
                default: return "GenericPiece";
            }
        }
    }
}
package frc.robot.utils.paths;

public class TrajectoryContainer {
    public final Path TWO_CENTER_RUSH;
    public final Path THREE_PIECE_CLOSE; //p123
    public final Path TWO_STAGE_RUSH;
    public final Path P_14;
    public final Path P_45;
    public final Path P_123PLUS;
    public final Path P_123PLUSTWO;
    
    public final Path P_1238;

    public final Path N_8;

    public TrajectoryContainer(String prefix) {
        TWO_CENTER_RUSH = new Path(prefix + " Two Center Rush");
        THREE_PIECE_CLOSE = new Path(prefix + " Three Piece Close");
        TWO_STAGE_RUSH = new Path(prefix + " Two Stage Rush");

        //New Naming convention
        //P for piece ### labling the pieces and the order they are grabed
        P_14 = new Path(prefix + " P14");
        P_123PLUS = new Path(prefix + " P123Plus");
        P_1238 = new Path(prefix + " P1238");
        P_45 = new Path(prefix + " P45");

        P_123PLUSTWO = new Path(prefix + " P123Plus2");

        N_8 = new Path(prefix + " N8");
    }
}

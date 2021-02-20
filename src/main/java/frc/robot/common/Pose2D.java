package frc.robot.common;

public class Pose2D extends Vector2D {

    public double ang;

    public Pose2D(Vector2D pos, double angle_input) {
        x = pos.x;
        y = pos.y;
        ang = angle_input;
    }

    public Pose2D(double x_input, double y_input, double angle_input) {
        x = x_input;
        y = y_input;
        ang = angle_input;
    }

    public Pose2D() {
        x = 0;
        y = 0;
        ang = 0;
    }

    public Vector2D getVector2D() {
        return new Vector2D(x, y, Vector2D.Type.CARTESIAN);
    }

    public void setVector2D(Vector2D vec) {
        x = vec.x;
        y = vec.y;
    }

    public Pose2D add(Pose2D toAdd) {
        return new Pose2D(getVector2D().add(toAdd.getVector2D()), ang + toAdd.ang);
    }

    public Pose2D add(Vector2D toAdd) {
        return this.add(new Pose2D(toAdd, 0));
    }

    public Pose2D subtract(Pose2D toSub) {
        return new Pose2D(getVector2D().subtract(toSub.getVector2D()), ang - toSub.ang);
    }

    public Pose2D scalarMult(double scalar){
        return new Pose2D(getVector2D().scalarMult(scalar), ang);
    }

    public Pose2D rotateAll(double radiansToRotate) {
        return new Pose2D(getVector2D().rotate(radiansToRotate), ang + radiansToRotate);
    }

    public Pose2D rotateAng(double radiansToRotate) {
        return new Pose2D(getVector2D(), ang + radiansToRotate);
    }

    public Pose2D rotateVec(double radiansToRotate){
        return new Pose2D(getVector2D().rotate(radiansToRotate), ang);
    }

    @Override
    public String toString() {
        return "(" + Util.roundHundreths(x) + ", " + Util.roundHundreths(y) + ", " + Util.roundHundreths(ang) + ")";
    }

}
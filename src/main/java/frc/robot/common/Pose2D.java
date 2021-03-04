package frc.robot.common;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

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

    public Pose2D exp(Pose2D twist) {
        double dx = twist.x;
        double dy = twist.y;
        double dtheta = twist.ang;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }
        Pose2D step = new Pose2D(dx * s - dy * c, dx * c + dy * s,
                new Vector2D(cosTheta, sinTheta, Type.CARTESIAN).getAngle());

        return this.add(step);
    }

    public Pose2D() {
        x = 0;
        y = 0;
        ang = 0;
    }

    public Vector2D getVector2D() {
        return new Vector2D(x, y, Vector2D.Type.CARTESIAN);
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, new Rotation2d(ang));
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

    public Pose2D scalarMult(double scalar) {
        return new Pose2D(getVector2D().scalarMult(scalar), ang);
    }

    public Pose2D rotateAll(double radiansToRotate) {
        return new Pose2D(getVector2D().rotate(radiansToRotate), ang + radiansToRotate);
    }

    public Pose2D rotateAng(double radiansToRotate) {
        return new Pose2D(getVector2D(), ang + radiansToRotate);
    }

    public Pose2D rotateVec(double radiansToRotate) {
        return new Pose2D(getVector2D().rotate(radiansToRotate), ang);
    }

    @Override
    public String toString() {
        return "(" + Util.roundHundreths(x) + ", " + Util.roundHundreths(y) + ", " + Util.roundHundreths(ang) + ")";
    }

}
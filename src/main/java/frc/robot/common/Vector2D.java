package frc.robot.common;

public class Vector2D {
    public double x;
    public double y;

    public enum Type {
        CARTESIAN, POLAR
    }

    public Vector2D(double magnitudeOrX, double directionOrY, Type vectorType) {
        if (vectorType == Type.CARTESIAN) {
            x = magnitudeOrX;
            y = directionOrY;
        } else {
            x = magnitudeOrX * Math.cos(directionOrY);
            y = magnitudeOrX * Math.sin(directionOrY);
        }
    }

    // unit vector
    public Vector2D(double direction) {
        x = Math.cos(direction);
        y = Math.sin(direction);
    }

    // zero vector
    public Vector2D() {
        x = 0;
        y = 0;
    }

    public Vector2D add(Vector2D valueToAdd) {
        return new Vector2D(this.x + valueToAdd.x, this.y + valueToAdd.y, Type.CARTESIAN);
    }

    public Vector2D subtract(Vector2D valueToSubtract) {
        return new Vector2D(this.x - valueToSubtract.x, this.y - valueToSubtract.y, Type.CARTESIAN);
    }

    public double dot(Vector2D toDot) {
        return this.x * toDot.x + this.y * toDot.y;
    }

    // pseudo cross product, returns double representing length of 3d vector
    public double pcross(Vector2D toCross) {
        return this.getMagnitude() * toCross.getMagnitude() * Math.sin(this.getAngle() - toCross.getAngle());
    }

    public Vector2D scalarAdd(double scalar) {
        return new Vector2D(this.x + scalar, this.y + scalar, Type.CARTESIAN);
    }

    public Vector2D scalarMult(double scalar) {
        return new Vector2D(this.x * scalar, this.y * scalar, Type.CARTESIAN);
    }

    public Vector2D scalarDiv(double scalar) {
        return new Vector2D(this.x / (double) scalar, this.y / (double) scalar, Type.CARTESIAN);
    }

    public Vector2D rotate(double radiansToRotate) {
        double sin = Math.sin(radiansToRotate);
        double cos = Math.cos(radiansToRotate);
        return new Vector2D(this.x * cos - this.y * sin, this.x * sin + this.y * cos, Type.CARTESIAN);
    }

    public Vector2D rotate90() {
        return new Vector2D(-this.y, this.x, Type.CARTESIAN);
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public double dist(Vector2D otherVec) {
        return this.subtract(otherVec).getMagnitude();
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }

    public boolean equals(Vector2D comparison) {
        double epsilon = 1e-10; // close enough to 10 decimal places
        return Math.abs(this.x - comparison.x) < epsilon && Math.abs(this.y - comparison.y) < epsilon;
    }

    public boolean equals(Vector2D comparison, double epsilon) {
        return Math.abs(this.x - comparison.x) < epsilon && Math.abs(this.y - comparison.y) < epsilon;
    }

    public String toString() {
        return "(" + Util.roundHundreths(x) + ", " + Util.roundHundreths(y) + ")";
    }

}

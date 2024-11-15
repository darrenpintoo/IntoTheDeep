package org.firstinspires.ftc.teamcode.utilities.math.linearalgebra;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Coordinate;
import java.util.function.Function;


public class Pose {
    private Coordinate theCoordinate;
    private double theFinalDirection;

    public Pose() {
        this.theCoordinate = new Coordinate(0,0);
        this.theFinalDirection = 0;
    }

    public Pose(Coordinate aCoordinate, double theFinalDirection) {
        this.theCoordinate = aCoordinate;
        this.theFinalDirection = theFinalDirection;
    }

    public Pose(Pose other) {
        this.theCoordinate = other.theCoordinate;
        this.theFinalDirection = other.theFinalDirection;
    }

    public org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Coordinate getCoordinate() {
        return theCoordinate;
    }

    public double getTheFinalDirection() {
        return theFinalDirection;
    }

    public void setTheFinalDirection(double theFinalDirection) {
        this.theFinalDirection = theFinalDirection;
    }

    public void add(Pose other) {
        this.theCoordinate.add( other.theCoordinate);
        this.theFinalDirection =  this.theFinalDirection + other.getTheFinalDirection();
    }

        //MEANING OF SUBSTRACTING ANGLES?
    public void substract(Pose other) {
        this.theCoordinate.substract( other.theCoordinate);
        this.theFinalDirection =  this.theFinalDirection - other.getTheFinalDirection();
    }

    public void rotate(double angle) {
        this.theCoordinate.rotate(angle);
        this.theFinalDirection = this.theFinalDirection + angle;
    }

    public void scale(double other) {
        this.theCoordinate.scale(other);
    }

    public void abs() {
        this.theCoordinate.abs();
        this.theFinalDirection =Math.abs(this.theFinalDirection);
    }

    public Pose map(Function<Double, Double> func) {
        this.theCoordinate.mapFunction(func);
        this.setTheFinalDirection(func.apply(this.getTheFinalDirection()));

        return this;
    }

    public double magnitude() {
        return theCoordinate.magnitude();
    }

}

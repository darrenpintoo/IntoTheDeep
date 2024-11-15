package org.firstinspires.ftc.teamcode.utilities.math.linearalgebra;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleConsumer;
import java.util.function.Function;


public class Coordinate {

        private double theX;
        private double theY;

        public Coordinate() {
            this(0, 0);
        }

        public Coordinate(double aX, double aY) {
            this.theX = aX;
            this.theY = aY;
        }

        public Coordinate(org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Coordinate aCoordinate) {
            this.theX = aCoordinate.theX;
            this.theY = aCoordinate.theY;
        }

        public double getX() {
            return theX;
        }

        public void setX(double aX) {
            this.theX = aX;
        }

        public double getY() {
            return theY;
        }

        public void setY(double aY) {
            this.theY = aY;
        }

        public void add(org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Coordinate aCoordinate) {
            this.setX(this.getX() + aCoordinate.getX());
            this.setY(this.getY() + aCoordinate.getY());    //Question why in this method using setX and sometimes direct?
        }

        public void substract(org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Coordinate aCoordinate) {
            this.setX(this.getX() - aCoordinate.getX());
            this.setY(this.getY() - aCoordinate.getY());
        }

        public void rotate(double angle) {
            double x = this.getX();
            double y = this.getY();

            this.setX(y * Math.cos(angle) - x * Math.sin(angle));
            this.setY(y * Math.sin(angle) + x * Math.cos(angle));
        }

        public void scale(double other) {
            this.setX(this.getX() * other);
            this.setY(this.getY() * other);
        }

        public void abs() {
            this.setX(Math.abs(this.getX()));
            this.setY(Math.abs(this.getY()));
        }

        public void mapFunction(Function<Double, Double> func) {
            this.setX(func.apply(this.getX()));
            this.setY(func.apply(this.getY()));
        }

        public double magnitude() {
            return Math.sqrt(theX * theX + theY * theY);
        }


    }


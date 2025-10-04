package org.firstinspires.ftc.teamcode.Tools;


public class Vector {

    private double x;
    private double y;

    public Vector(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void setX(double x){
        this.x = x;
    }

    public void setY(double y){
        this.y = y;
    }

    public double magnitude (){
        return Math.sqrt(x*x+y*y);
    }

    public Vector normalize(){
        double mag = magnitude();
        if(mag==0){return new Vector(0,0);}

        return new Vector(x/mag,y/mag);
    }

    public Vector add(Vector other) {
        return new Vector(this.x + other.x, this.y + other.y);
    }


    public Vector subtract(Vector other) {
        return new Vector(this.x - other.x, this.y - other.y);
    }


    public Vector scale(double scalar) {
        return new Vector(this.x * scalar, this.y * scalar);
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }


}
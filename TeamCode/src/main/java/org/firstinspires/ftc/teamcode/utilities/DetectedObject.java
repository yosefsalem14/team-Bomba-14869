package org.firstinspires.ftc.teamcode.utilities;

public class DetectedObject {
    private int pos;
    private ObjectPositions ID;
    private double angle;
    public DetectedObject(int pos,double angle){
        this.pos = pos;
        this.angle = angle;
        this.ID = ObjectPositions.UNKNOWN;
    }
    public DetectedObject(){
        this.pos = 0;
        this.angle = 0;
        this.ID = ObjectPositions.CEMTER;
    }
    public int getPos(){
        return this.pos;
    }
    public void setID(ObjectPositions newID){
        this.ID = newID;
    }
    public ObjectPositions getID(){
        return this.ID;
    }
    public double getAngle(){
        return this.angle;
    }
    public void setAngle(double ang){
        this.angle = ang;
    }
    public void set(DetectedObject object){
        this.pos = object.pos;
        this.angle = object.angle;
        this.ID = object.ID;
    }
}

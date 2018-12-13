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

}

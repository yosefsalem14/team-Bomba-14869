package org.firstinspires.ftc.teamcode.utilities;

/////////////////// COULD BE DONE //////////////
///////////////////       :>      //////////////

/*

PID class, follows the PID formula from wiki-pedia!
 */
public class PID  {
    private double KP;
    private double KI;
    private double KD;
    private double errorSum;
    private double currentTime;
    private double previousTime;
    private double previousError;
    private double P,I,D,DT;
    /*
        just a constructor, sets all the K values and initializes
        the other variables(mostly sets them to 0)
     */
    public PID(double KP,double KI,double KD){
        this.P = 0;
        this.I = 0;
        this.D = 0;
        this.DT = 0;
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.errorSum = 0;
        this.currentTime = System.currentTimeMillis();
        this.previousTime = 0;
    }



    /*
    returns the error between 2 numbers, ***MIGHT GET REMOVED***
     */
    public double getError(double current,double target){

        return target-current;
    }


    /*
        the heart of the PID algorithm,
        takes an error passes it through the weighted-sum and spits out
        the right power
     */
    public double getPower(double error){
        errorSum += error;
        currentTime = System.currentTimeMillis();
         this.DT= currentTime - previousError;
        if(this.DT>=100) {
            this.P = error;
            this.I = errorSum;
            this.D = (P - previousError) / DT;
            previousError = error;
            previousTime = System.currentTimeMillis();
        }
        return this.P*KP + this.I*KI + this.D*KD;
    }




    /*
        return all the K values
     */
    public double getKP(){
        return KP;
    }
    public double getKI(){
        return KI;
    }
    public double getKD(){
        return KD;
    }



    /*
        Change the K values, **MIGHT GET REMOVED**
     */
    public void doubleSetCoefficients(double KP,double KI,double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }



    /*
        resets all the variables, this is used to make the PID object reusable!
     */
    public void reset(){
        this.previousError = 0;
        this.currentTime = 0;
        this.errorSum = 0;
        this.previousTime = 0;
    }
}

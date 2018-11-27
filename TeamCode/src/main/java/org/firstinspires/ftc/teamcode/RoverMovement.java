package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/*
    ///////main TeleOP class/////
    this will be used for the main robot
    TODO:
    this is almost done, just fix the comments!
 */

@TeleOp(name="rover movement",group="movement")

public class RoverMovement extends LinearOpMode {
    robot rover = new robot();

    @Override
    public void runOpMode(){//throws InterruptedException {
        rover.init(hardwareMap);

        //define controller input variables
            double move =            0.0;
            boolean latchOpen = false;
            double armMove =         0.0;
            double turn =            0.0;
            double leftBumper =      0.0;
            double rightBumper =     0.0;
            double strafe =          0.0;
            double leftTrigger =     0.0;
            double rightTrigger =    0.0;
            double strmove =         0.0;
            double com2LeftBumper =  0.0;
            double com2RightBumper = 0.0;
            double collect =         0.0;
        for(int i =0;i<rover.latches.length;i++){
            rover.latches[i].setDirection(Servo.Direction.FORWARD);
        }
        boolean once = true;
        telemetry.addData("status","ready for start!");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {

            //get all the variables:
                //parse the bumper clicks
                    leftBumper =      (gamepad1.left_bumper ? 1 : 0);
                    rightBumper =     (gamepad1.right_bumper ? 1 : 0);
                    com2LeftBumper =  (gamepad2.left_bumper ? 1 : 0);
                    com2RightBumper = (gamepad2.right_bumper ? 1 : 0);



                //parse the trigger pushes
                    leftTrigger =   gamepad1.left_trigger;
                    rightTrigger =  gamepad1.right_trigger;


                //calculate the turn & strafe & move factor

                    move =     (-gamepad1.left_stick_y)*rover.movePower;
                    turn =     (rightTrigger - leftTrigger)*rover.turnPower;
                    strafe =   (rightBumper - leftBumper)*rover.strafePower;



                //calculate the arm & stetcher & collector move factor+
                    armMove = -gamepad2.left_stick_y;
                    strmove = gamepad2.right_stick_y;
                    collect = com2RightBumper - com2LeftBumper;

                //open and close the latches
                    boolean APressed = gamepad2.x;
                if(APressed){
                    if(once) {
                        latchOpen = !latchOpen;
                        once=!once;
                    }
                }else{
                    once = true;
                }

            /*
             * MAIN MOTOR MOVEMENT
             *
              */
            //calculate the specific motor power
                double leftBack =   Range.clip(move - turn + strafe,
                        -1,1);
                double leftFront =  Range.clip(move - turn - strafe,
                        -1,1);
                double rightBack =  Range.clip(move + turn - strafe,
                        -1,1);
                double rightFront = Range.clip(move + turn + strafe,
                        -1,1);


            //do the movements:
            rover.mainMotors[0].setPower(leftBack);
            rover.mainMotors[1].setPower(rightBack);
            rover.mainMotors[2].setPower(leftFront);
            rover.mainMotors[3].setPower(rightFront);




            /*
              PICKING UP MECHANISM MOVEMENT
             */
            //arm  movement
            for (int i = 0; i < rover.armMotors.length; i++) {
                    rover.armMotors[i].setPower(armMove *
                            rover.armPower);
            }

            //collector movement
            rover.collector.setPower(-collect *
                    rover.collectPower);

            //stretcher movement
            rover.stretcher.setPower(strmove *
                    rover.stretchPower);
            for(int i =0;i<rover.latches.length;i++) {
                if (latchOpen) {
                    if(i%2==0) {
                        rover.latches[i].setPosition(0);
                    }else{
                        rover.latches[i].setPosition(1);
                    }
                } else {
                    if(i%2==0) {
                        rover.latches[i].setPosition(1);
                    }else{
                        rover.latches[i].setPosition(0);
                    }
                }
            }
            //output
            if(armMove!=0)
                telemetry.addData("player2_moves",
                        "moving arm!");
            else if(collect!=0)
                telemetry.addData("player2_moves",
                        "moving collector!");
            else if(strmove!=0)
                telemetry.addData("player2_moves",
                        "moving stretcher!");
            else
                telemetry.addData("player2_moves",
                        "idle");



            //show all the input values
            telemetry.addData("move",
                    "move= " + formatString(move));

            telemetry.addData("turn",
                    "turn value= " +
                           formatString(turn));

            telemetry.addData("strafe",
                    "strafe value= " +
                            formatString(strafe));

            telemetry.addData("arm",
                    "arm value= " +
                            formatString(armMove));

            telemetry.addData("stretcher",
                    "stretcher value= " +
                            formatString(strmove));

            telemetry.addData("collector",
                    "collector value= " +
                            formatString(collect));

            telemetry.addLine();


            telemetry.addData("status",
                    "running");
            telemetry.update();
        }
    }
    public String formatString(double value){
        return (Math.abs(Math.floor(value*100)) + "%");
    }
}



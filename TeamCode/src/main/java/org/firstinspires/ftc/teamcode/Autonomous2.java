/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto2_v3.2", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class Autonomous2 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor left = null;
    DcMotor right = null;
    final double inToEnc = 360.0 / Math.PI;
    final double degToEnc = 16;  //placeholder
    Telemetry.Item leftEnc, rightEnc, leftSpd, rightSpd;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        left  = hardwareMap.dcMotor.get("L");
        right = hardwareMap.dcMotor.get("R");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        resetEnc();

        leftEnc = telemetry.addData("left encoder:", 0);
        rightEnc = telemetry.addData("right encoder:", 0);
        leftSpd = telemetry.addData("left motor speed:", 0);
        rightSpd = telemetry.addData("right motor speed:", 0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            /*forward(2, 0.3);
            turnLeft(43, 0.5);
            forward(55, 0.7);
            turnLeft(43, 0.5);
            forward(3, 0.3);
            //beacon
            backward(3, 0.3);
            turnRight(87, 0.5);
            forward(45, 0.6);
            turnLeft(87, 0.5);
            forward(3, 0.3);
            //beacon
            backward(3, 0.3);*/

            resetEnc();

            //forward(60, 0.7);
            backward(30, 0.6);
            //TODO: the forward and backward methods work independently of each other, but only the first runs if they're chained - fix this
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    /**
     * Precondition: all values >= 0
     *
     * Drive robot forward a given distance at a given speed
     *
     * @param dist distance in inches
     * @param spd speed of motors, within [-1.0, 1.0]
     */
    void forward(int dist, double spd) throws InterruptedException{
        //resetEnc();

        int initL = left.getCurrentPosition();
        int initR = right.getCurrentPosition();
        int targL = (int)(initL + (dist * inToEnc));
        int targR = (int)(initR + (dist * inToEnc));
        int thresh = (int)(dist * inToEnc * 0.15);

        while(opModeIsActive() && left.getCurrentPosition() <= targL && right.getCurrentPosition() <= targR){
            double currLPos = left.getCurrentPosition();
            double currRPos = right.getCurrentPosition();

            if(currLPos <= initL + thresh && currRPos <= initR + thresh){
                left.setPower(Range.scale(currLPos, initL, initL + thresh, 0.1, spd));
                right.setPower(Range.scale(currRPos, initR, initR + thresh, 0.1, spd));
            }
            else if(currLPos > initL + thresh && currLPos < targL - thresh && currRPos > initR + thresh && currRPos < targR - thresh){
                left.setPower(spd);
                right.setPower(spd);
            }
            else{
                left.setPower(Range.scale(currLPos, targL - thresh, targL, spd, 0));
                right.setPower(Range.scale(currRPos, targR - thresh, targR, spd, 0));
            }
            leftEnc.setValue(currLPos);
            rightEnc.setValue(currRPos);
            leftSpd.setValue(left.getPower());
            rightSpd.setValue(right.getPower());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }

    private void updateEncoders() throws InterruptedException{
        while(opModeIsActive() && left.isBusy() || right.isBusy()) {
            leftEnc.setValue(left.getCurrentPosition());
            rightEnc.setValue(right.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }

    /**
     * Precondition: all values >= 0
     *
     * Drive robot backwards a given distance at a given speed
     *
     * @param dist distance in inches
     * @param spd speed of motors, within [-1.0, 1.0]
     */
    void backward(int dist, double spd) throws InterruptedException{
        //resetEnc();

        int initL = left.getCurrentPosition();
        int initR = right.getCurrentPosition();
        int targL = (int)(initL - (dist * inToEnc));
        int targR = (int)(initR - (dist * inToEnc));
        int thresh = (int)(dist * inToEnc * 0.15);

        while(opModeIsActive() && left.getCurrentPosition() >= targL && right.getCurrentPosition() >= targR){
            double currLPos = left.getCurrentPosition();
            double currRPos = right.getCurrentPosition();

            if(currLPos >= initL - thresh && currRPos >= initR - thresh){
                left.setPower(-Range.scale(currLPos, initL, initL - thresh, 0.1, spd));
                right.setPower(-Range.scale(currRPos, initR, initR - thresh, 0.1, spd));
            }
            else if(currLPos < initL - thresh && currLPos > targL + thresh && currRPos < initR - thresh && currRPos > targR + thresh){
                left.setPower(-spd);
                right.setPower(-spd);
            }
            else{
                left.setPower(-Range.scale(currLPos, targL + thresh, targL, spd, 0));
                right.setPower(-Range.scale(currRPos, targR + thresh, targR, spd, 0));
            }
            leftEnc.setValue(currLPos);
            rightEnc.setValue(currRPos);
            leftSpd.setValue(left.getPower());
            rightSpd.setValue(right.getPower());
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }

    //TODO: fix turn methods to account for rotation in degrees (or radians), not inches - will require testing

    /**
     * Precondition: all values >= 0
     *
     * Rotates the robot counterclockwise
     *
     * @param deg amount of degrees to turn
     * @param spd speed of turning, [-1.0, 1.0]
     */
    void turnLeft(int deg, double spd) throws InterruptedException{
        if(opModeIsActive()) {
            int dL = left.getCurrentPosition() - (int) (deg * degToEnc);
            int dR = right.getCurrentPosition() + (int) (deg * degToEnc);

            left.setTargetPosition(dL);
            right.setTargetPosition(dR);
            left.setPower(spd);
            right.setPower(spd);

            updateEncoders();
        }
    }

    /**
     * Precondition: all values >= 0
     *
     * Rotates the robot clockwise
     *
     * @param deg amount of degrees to turn
     * @param spd speed of turning, [-1.0, 1.0]
     */
    void turnRight(int deg, double spd) throws InterruptedException{
        if(opModeIsActive()) {
            int dL = left.getCurrentPosition() + (int) (deg * degToEnc);
            int dR = right.getCurrentPosition() - (int) (deg * degToEnc);

            left.setTargetPosition(dL);
            right.setTargetPosition(dR);
            left.setPower(spd);
            right.setPower(spd);

            updateEncoders();
        }
    }

    private void resetEnc(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

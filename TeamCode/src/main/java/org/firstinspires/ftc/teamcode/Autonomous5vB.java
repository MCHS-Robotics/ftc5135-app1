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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;


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

@Autonomous(name="Auto v6.5.1 B blu", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class Autonomous5vB extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor left = null;
    DcMotor right = null;
    private DcMotor shootA = null;
    private DcMotor shootB = null;
    final double IN_TO_ENC = 360.0 / Math.PI;
    final double DEG_TO_ENC = 124.0/9;  //placeholder
    //Telemetry.Item leftEnc, rightEnc;
    ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ColorSensor sensorRGB;
    AnalogInput linSens;
    DeviceInterfaceModule cdim;
    private CRServo bacon = null;
    private final boolean TEAM = false;  //true=red team, false = blue team
    private boolean complete = false;

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
        shootA = hardwareMap.dcMotor.get("A");
        shootB = hardwareMap.dcMotor.get("B");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        left.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        right.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*leftEnc = */telemetry.addData("left encoder:", 0);
        /*rightEnc =*/ telemetry.addData("right encoder:", 0);
        telemetry.update();

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        sensorRGB = hardwareMap.colorSensor.get("color");
        linSens = hardwareMap.analogInput.get("line");

        bacon = hardwareMap.crservo.get("bcn");
        bacon.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double sTime = runtime.time();

        // run until the end of the match (driver presses STOP)
        //if (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //test auto - move in a square
            /*for(int i = 0; i < 4; i++){
                forward(42, 0.7);
                turnRight(90, 0.5);
            }*/

            //red team, closer to corner vortex - beacons only
           /* backwards(2, 0.3);
            turnRight(38, 0.5);
            backwards(45, 0.7);
            turnLeft(38, 0.5);
            backwards(3, 0.3);
            checkLine(-0.2);
            hitBeacon();
            //backwards(3, 0.3);
            //turnRight(87, 0.5);
            backwards(45, 0.6);
            //turnLeft(87, 0.5);
            //forward(3, 0.3);
            checkLine(-0.2);
            hitBeacon();*/
            //backwards(3, 0.3);

            checkLine(-0.07);

            //complete = true;

            //cap ball, park mid in a diagonal
            //forward(24, 0.3);
            //turnRight(30, 0.3);

            //red close to vortex, partial park on corner
            /*forward(14, 0.5);
            turnLeft(43, 0.35);
            forward(18, 0.5);
            turnLeft(87, 0.35);
            forward(5, 0.6);
            shootBall(1.0);
            turnLeft(180, 0.3);
            forward(7, 0.55);*/

            //go forward based on time
            /*left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //sleep(10000);
            //either this or just a simple "sleep"

            while(runtime.time() < sTime + 10){
                left.setPower(0);
                right.setPower(0);
            }
            if(runtime.time() < sTime + 13)
                timedForward(3000);*/

            /*if(complete)
                requestOpModeStop();*/
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        //}
    }


    /**
     * Resets encoders, then changes Runmode to RUN_TO_POSITION
     */
    private void setMotorRtP(){
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * changes Runmode to RUN_USING ENCODER
     */
    private void turnOffRtP(){
        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if(opModeIsActive()) {

            int dL = left.getCurrentPosition() + (int) (dist * IN_TO_ENC);
            int dR = right.getCurrentPosition() + (int) (dist * IN_TO_ENC);

            left.setTargetPosition(dL);
            right.setTargetPosition(dR);

            setMotorRtP();

            left.setPower(spd);
            right.setPower(spd);

            updateEncoders();

            turnOffRtP();
        }
    }

    private void updateEncoders() throws InterruptedException{
        telemetry.clearAll();
        while(opModeIsActive() && (left.isBusy() && right.isBusy())) {  //should this be L&&R or L||R?
            telemetry.addData("Target Positions", "L: %d  R: %d", left.getTargetPosition(), right.getTargetPosition());
            telemetry.addData("Current Position", "L: %d  R: %d", left.getCurrentPosition(), right.getCurrentPosition());
            telemetry.addData("Motor Power", "L: %7f  R: %7f", left.getPower(), right.getPower());
            telemetry.update();

            idle();
        }
    }

    void timedForward(double ms) throws InterruptedException{
        et.reset();
        double startTime = et.time();
        double endTime = startTime + ms;
        while(et.time() < endTime){
            left.setPower(0.4);
            right.setPower(0.4);
            //idle();
        }
        left.setPower(0);
        right.setPower(0);
    }

    void timedBeacon(double ms) throws InterruptedException{
        et.reset();
        double startTime = et.time();
        double endTime = startTime + ms;
        while(et.time() < endTime){
            bacon.setPower(-0.4);
            //idle();
        }
        bacon.setPower(0);
    }

    void timedBeaconBack(double ms) throws InterruptedException{
        et.reset();
        double startTime = et.time();
        double endTime = startTime + ms;
        while(et.time() < endTime){
            bacon.setPower(0.4);
            //idle();
        }
        bacon.setPower(0);
    }

    /**
     * Precondition: all values >= 0
     *
     * Drive robot backwards a given distance at a given speed
     *
     * @param dist distance in inches
     * @param spd speed of motors, within [-1.0, 1.0]
     */
    void backwards(int dist, double spd) throws InterruptedException{
        if(opModeIsActive()) {
            setMotorRtP();

            int dL = left.getCurrentPosition() - (int) (dist * IN_TO_ENC);
            int dR = right.getCurrentPosition() - (int) (dist * IN_TO_ENC);

            left.setTargetPosition(dL);
            right.setTargetPosition(dR);
            left.setPower(spd);
            right.setPower(spd);

            updateEncoders();
        }
        turnOffRtP();
    }

    /**
     * Precondition: all values >= 0
     *
     * Rotates the robot counterclockwise
     *
     * @param deg amount of degrees to turn
     * @param spd speed of turning, [-1.0, 1.0]
     */
    void turnRight(int deg, double spd) throws InterruptedException{
        if(opModeIsActive()) {
            setMotorRtP();

            int dL = left.getCurrentPosition() - (int) (deg * DEG_TO_ENC);
            int dR = right.getCurrentPosition() + (int) (deg * DEG_TO_ENC);

            left.setTargetPosition(dL);
            right.setTargetPosition(dR);
            left.setPower(spd);
            right.setPower(spd);

            updateEncoders();
        }
        turnOffRtP();
    }

    /**
     * Precondition: all values >= 0
     *
     * Rotates the robot clockwise
     *
     * @param deg amount of degrees to turn
     * @param spd speed of turning, [-1.0, 1.0]
     */
    void turnLeft(int deg, double spd) throws InterruptedException{
        if(opModeIsActive()) {
            setMotorRtP();

            int dL = left.getCurrentPosition() + (int) (deg * DEG_TO_ENC);
            int dR = right.getCurrentPosition() - (int) (deg * DEG_TO_ENC);

            left.setTargetPosition(dL);
            right.setTargetPosition(dR);
            left.setPower(spd);
            right.setPower(spd);

            updateEncoders();
        }
        turnOffRtP();
    }

    /**
     * Checks the color of the half of the beacon that the Adafruit RGB Sensor is facing.
     * Gets the red value and the blue value, and find the difference.
     * Returns positive if red, negative if blue.
     *
     * @return red - blue the difference between the aforementioned values as scanned by the sensor
     */
    int scanBeacon(){
        int red = sensorRGB.red();
        int blue = sensorRGB.blue();

        return red - blue;
    }

    /**
     * Operates the beacon-hitting arm
     */
    void hitBeacon() throws InterruptedException{
        bacon.setPower(0);
        int bcnCol = scanBeacon();
        //TODO: make one version of this for team Red and one for team Blue-servo will react differently in either case
        //also depends on sensor placement
        if(bcnCol > 0){
            //facing red beacon
            if(TEAM) {    //if red team
                //bacon.setPosition(0.7);//check this position; swing right
                timedBeacon(1000);
                timedBeaconBack(900);
            }
            else{    //if blue team
                forward(6, 0.2);
                timedBeacon(1000);
                timedBeaconBack(900);
            }
        }
        else{
            //facing blue beacon
            if(TEAM) {    //if red team:
                forward(6, 0.2);
                timedBeacon(1000);
                timedBeaconBack(900);
            }
            else {    //if blue team:
                timedBeacon(1000);
                timedBeaconBack(900);
            }
        }
    }

    /**
     * Operates the ball shooter
     * speed > 0 launches out
     * speed < 0 pulls balls in
     *
     * @param speed speed that the shooter runs at
     */
    void shootBall(double speed){
        double sTime = et.milliseconds();
        double eTime = sTime + 3500;
        while(sTime < eTime) {
            shootA.setPower(speed);
            shootB.setPower(speed);
            left.setPower(-0.3);
            right.setPower(-0.3);
            sTime = et.milliseconds();
        }
    }

    void checkLine(double pwrDir){
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //check initial readings
        double voltage = linSens.getVoltage();  //should start scan before the line
        double newVolt = voltage;

        //go forward slowly while no significant change in readings
        while(!(newVolt < voltage - 0.6)){
            newVolt = linSens.getVoltage();
            telemetry.addData("Voltage", newVolt);
            left.setPower(pwrDir);
            right.setPower(pwrDir);
        }
        left.setPower(0);
        right.setPower(0);
    }

}

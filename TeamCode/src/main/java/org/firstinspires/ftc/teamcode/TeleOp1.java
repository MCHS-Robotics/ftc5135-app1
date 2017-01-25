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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp v1.6.2", group="TeleOp")  // @Autonomous(...) is the other common choice
//@Disabled
public class TeleOp1 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor left = null;
    private DcMotor right = null;

    private DcMotor shootA = null;
    private DcMotor shootB = null;

    private CRServo bacon = null;

    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;

    ElapsedTime et = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private final boolean TEAM = true;  //true=red team, false = blue team



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        left  = hardwareMap.dcMotor.get("L");
        right = hardwareMap.dcMotor.get("R");

        shootA = hardwareMap.dcMotor.get("A");
        shootB = hardwareMap.dcMotor.get("B");

        bacon = hardwareMap.crservo.get("bcn");
        bacon.setPower(0);

        //cdim = hardwareMap.deviceInterfaceModule.get("dim");
        //sensorRGB = hardwareMap.colorSensor.get("color");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        left.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        shootA.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        shootB.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){
        telemetry.addData("Status", "Running: " + runtime.toString());

        //tank drive
        left.setPower(Range.clip(gamepad1.left_stick_y+gamepad1.right_stick_x, -1, 1));
        right.setPower(Range.clip(gamepad1.left_stick_y-gamepad1.right_stick_x, -1, 1));

        //shooter--press buttons for speed multiplier
        if(gamepad2.dpad_up){
            if(gamepad2.a){
                shootA.setPower(0.625);
                shootB.setPower(0.625);
            }
            else if(gamepad2.b){
                shootA.setPower(0.75);
                shootB.setPower(0.75);
            }
            else if(gamepad2.x){
                shootA.setPower(0.875);
                shootB.setPower(0.875);
            }
            else if(gamepad2.y){
                shootA.setPower(1);
                shootB.setPower(1);
            }
            else{
                shootA.setPower(0.5);
                shootB.setPower(0.5);
            }
        }
        else if(gamepad2.dpad_down){
            if(gamepad2.a){
                //special slowing modifier, not speed up
                shootA.setPower(-0.3);
                shootB.setPower(-0.3);
            }
            else if(gamepad2.b){
                shootA.setPower(-0.75);
                shootB.setPower(-0.75);
            }
            else if(gamepad2.x){
                shootA.setPower(-0.875);
                shootB.setPower(-0.875);
            }
            else if(gamepad2.y){
                shootA.setPower(-1);
                shootB.setPower(-1);
            }
            else{
                shootA.setPower(-0.5);
                shootB.setPower(-0.5);
            }
        }
        else{
            shootA.setPower(0);
            shootB.setPower(0);
        }

        //straight-up beacon hitter actuator
        if(gamepad2.right_trigger >= 0.05){
            bacon.setPower(-gamepad2.right_trigger);    //push out
        }
        else if(gamepad2.left_trigger >= 0.05){
            bacon.setPower(gamepad2.left_trigger);    //pull in
        }
        else{
            bacon.setPower(0);
        }

        //test for the autonomous beacon hitter
        /*if(gamepad2.right_bumper)
            hitBeacon();
        */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        left.setPower(0);
        right.setPower(0);
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
    /*void hitBeacon() throws InterruptedException{
        bacon.setPower(0);
        int bcnCol = scanBeacon();
        //TODO: make one version of this for team Red and one for team Blue-servo will react differently in either case
        //also depends on sensor placement
        if(bcnCol > 0){
            //facing red beacon
            if(TEAM) {    //if red team
                //bacon.setPosition(0.7);//check this position; swing right
                //timedBeacon(2000);
                bacon.setPower(0.5);
                Thread.sleep(3000);
                bacon.setPower(-0.5);
                wait(2500);
                bacon.setPower(0);
            }
            else{    //if blue team
                //forward(6, 0.2);
                //timedBeaconBack(2000);
            }
        }
        else{
            //facing blue beacon
            if(TEAM) {    //if red team:
                //forward(6, 0.2);
                //timedBeacon(2000);
            }
            else {    //if blue team:
                //timedBeaconBack(2000);
            }
        }
    }*/

    void timedBeacon(double ms) throws InterruptedException{
        et.reset();
        double startTime = et.time();
        double endTime = startTime + ms;
        while(et.time() < endTime){
            bacon.setPower(-0.4);
            //idle();
        }
        left.setPower(0);
        right.setPower(0);
    }

    void timedBeaconBack(double ms) throws InterruptedException{
        et.reset();
        double startTime = et.time();
        double endTime = startTime + ms;
        while(et.time() < endTime){
            bacon.setPower(0.4);
            //idle();
        }
        left.setPower(0);
        right.setPower(0);
    }

}

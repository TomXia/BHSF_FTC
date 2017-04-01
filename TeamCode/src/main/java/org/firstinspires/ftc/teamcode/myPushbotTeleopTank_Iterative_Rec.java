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


import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.DataOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.BlockingQueue;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Debug: Testor", group="Debug")
//@Disabled
public class myPushbotTeleopTank_Iterative_Rec extends OpMode{

    /* Declare OpMode members. */
    final private double obsThreshold =0.15;

    private HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware

    private shootThread shoot;
    private collector collect;
    private shoot_servo reloader;
    private releaseLadder dropper = new releaseLadder(robot);
    private double rx,ry,x,y;
    private boolean is_Up;
    private BlockingQueue que;
    private Gamepad bgp;
    private CommandInfo ci = new CommandInfo();
    private ElapsedTime et;
    private DataOutputStream out;
    private FileOutputStream fos = null;
    public Long last = 0l;
    subAuto sua;
                                                         // could also use HardwarePushbotMatrix class.
//    double          clawOffset  = 0.0 ;                  // Servo mid position
//    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        sua = new subAuto(robot,null,1);
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            fos = hardwareMap.appContext.openFileOutput("Temp.txt", Context.MODE_WORLD_READABLE);
        }catch (FileNotFoundException e){
            telemetry.addData("Failed while","create");
            e.printStackTrace();
        }
        out = new DataOutputStream(fos);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        sua.t=telemetry;
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
        robot.gyro.resetZAxisIntegrator();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        rx=gamepad1.left_stick_x;
        ry=gamepad1.left_trigger-gamepad1.right_trigger;
        x=(rx==0)?1:Math.abs(rx)/rx;
        y=(ry==0)?1:Math.abs(ry)/ry;
        x*=1-Math.sqrt(1-(rx*rx));
        y*=1-Math.sqrt(1-(ry*ry));
        robot.pushGamepad(x, y);
        //sua.ultrasonicgo(true);
        /*robot.pushLight.setPosition(gamepad1.right_stick_x);
        if(gamepad1.right_bumper)
        {
            robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while(gamepad1.right_bumper){}
        }*/
        //telemetry.addData("motor: ","%d",robot.l1.getCurrentPosition());
        telemetry.addData("speed:","%f",robot.l1.getPower());
        telemetry.addData("deg L1: ","%d", robot.l1.getCurrentPosition());
        telemetry.addData("deg L2: ","%d", robot.l2.getCurrentPosition());
        telemetry.addData("deg R1: ","%d", robot.r1.getCurrentPosition());
        telemetry.addData("deg R2: ","%d", robot.r2.getCurrentPosition());
        telemetry.addData("LightSensorL: ","%f",robot.le.getLightDetected());
        telemetry.addData("LightSensorR:","%f",robot.re.getLightDetected());
        telemetry.addData("UltrasonicSensorf:","%f",robot.ulsf.getUltrasonicLevel());
        telemetry.addData("UltrasonicSensorb:","%f",robot.ulsb.getUltrasonicLevel());
        //telemetry.addData("LS","%f",robot.ls.getLightDetected());
        telemetry.addData("ods","%f",robot.ods.getLightDetected());
       // telemetry.addData("servoPush","%f",robot.pushLight.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        try {
            fos.close();
            out.close();
        }catch (IOException e){
            telemetry.addData("Failed while","close");
            e.printStackTrace();
        }

        robot.stop();
    }

}

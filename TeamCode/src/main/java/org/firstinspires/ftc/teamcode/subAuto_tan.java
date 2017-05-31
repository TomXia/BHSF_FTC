package org.firstinspires.ftc.teamcode;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import java.lang.Math;

/**
 * Created by Administrator on 2017/4/13.
 */

public class subAuto_tan extends subAuto {
    private int distance_First_goShoot = 4100;
    private int distance_Right_goShoot = 1400;
    private int distance_goBucket_goSh = distance_First_goShoot/2+500 + (destColour-1)*200;

    private int distance_pushBall_turn = 600 + destColour*1375;
    private int distance_pushBall_go = 3500;
    private int distance_pushBall_back = 3500;

    subAuto_tan(HardwarePushbot  r, LinearVisionOpMode op, double q){
        super(r,op,q);
    }

    public void goShoot(){
        pushDeg(distance_First_goShoot,0,(-0.5)*qSpeed,true);
        pushDeg(distance_Right_goShoot,0.5,0,true);
       //pushDeg(distance_goBucket_goSh,0,-0.5 + destColour*1.0,true);
        robot.resetMotors();
        robot.pushGamepad(0,-0.5 + destColour*1.0);
        while(opmode == null || opmode.opModeIsActive()){
            if(Math.abs(robot.l2.getCurrentPosition()*Q) >= distance_goBucket_goSh) break;
        }
        robot.pushGamepad(0,0);
        shootBall(2);
    }

    void pushBall(){
        if(destColour==BEACON_COLOUR_BLUE)
            pushDeg(Math.abs((destColour-1)*distance_pushBall_back),0,0.9,true);
        else
            pushDeg(distance_pushBall_back - 1000,0,-0.9,true);
        pushDeg(distance_pushBall_turn,-0.5,0,true);
        pushDeg(distance_pushBall_go - destColour * 1600,0,-0.8*qSpeed,false);
    }
}

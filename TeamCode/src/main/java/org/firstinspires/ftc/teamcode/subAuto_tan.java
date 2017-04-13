package org.firstinspires.ftc.teamcode;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import java.lang.Math;

/**
 * Created by Administrator on 2017/4/13.
 */

public class subAuto_tan extends subAuto {
    private int distance_First_goShoot = 4100;
    private int distance_Right_goShoot = 1860;
    private int distance_goBucket_goSh = distance_First_goShoot/2+500 + (destColour-1)*200;

    private int distance_pushBall_turn = 600 + destColour*1675;
    private int distance_pushBall_go = 4000;
    private int distance_pushBall_back = Math.abs((destColour-1)*5000);

    subAuto_tan(HardwarePushbot  r, LinearVisionOpMode op, double q){
        super(r,op,q);
    }

    public void goShoot(){
        pushDeg(distance_First_goShoot,0,(-0.5)*qSpeed,true);
        pushDeg(distance_Right_goShoot,0.5,0,true);
        pushDeg(distance_goBucket_goSh,0,-0.5 + destColour*1.0,true);
        shootBall(2);
    }

    void pushBall(){
        pushDeg(Math.abs((destColour-1)*distance_pushBall_back),0,0.9,true);
        pushDeg(distance_pushBall_turn,-0.5,0,true);
        pushDeg(distance_pushBall_go - destColour * 1000,0,-0.8*qSpeed,false);
    }
}

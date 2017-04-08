package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;


/**
 * Created by Administrator on 2017/3/28.
 */

public class subAuto_right extends subAuto {
    private int distance_First_goShoot = 3900;
    private int distance_pushBall_go = 3300;
    private int distance_Right_goShoot = 1750;
    private int distance_pushBall_back = 4000;
    private int distance_pushBall_red = 3000;
    subAuto_right(HardwarePushbot  r, LinearVisionOpMode op, double q){
        super(r,op,q);
    }
    
    void goShoot(){
        pushDeg(distance_First_goShoot,0,(-0.5)*qSpeed,true);
        pushDeg(distance_Right_goShoot,0.5,0,true);
        shootBall(2);
    }
    void pushBall(){
        pushDeg(Math.abs((destColour-1)*distance_pushBall_back),0,0.9,true);
        if(destColour==1)pushDeg(distance_pushBall_red,0,qSpeed*(-0.6),true);
        pushDeg(610 - destColour*65 + destColour*1750,-0.5,0,true);
        pushDeg(distance_pushBall_go,0,-0.8,false);
    }


}

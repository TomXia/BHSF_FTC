package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Administrator on 2017/3/14.
 */

public class subAuto {
    HardwarePushbot robot;
    private shoot_servo reloader;
    public LinearOpMode opmode;
    public double qSpeed;
    public  subAuto(HardwarePushbot r, LinearOpMode op, double q)
    {
        this.robot = r;
        opmode = op;
        reloader = new shoot_servo(robot);
        qSpeed = q;
    }
    public void shootBall(int times){
        for(int i=0;i<times;++i)
        {
            robot.miniGun.setPower(0.58);
            while (!robot.TCH.isPressed()) {
            }
            while (robot.TCH.isPressed()) {
            }
            while (!robot.TCH.isPressed()) {
            }
            robot.miniGun.setPower(0);
            if (i==0 && !reloader.isAlive())            reloader.start();
            try {
                Thread.sleep((i==0 ? 1:0) * 1500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public void ultrasonicgo(Boolean isForward)
    {
//            while (robot.ods.getLightDetected() < 0.3) {
                double len = 16.0;
                double lenth = (robot.uls.getUltrasonicLevel());
                double delta = 0.02 * (lenth - len);
                double x, y;


                if(isForward){
                    if ((8 * delta - delta * delta) < 0) {
                        delta = (Math.sqrt(Math.abs(8 * delta - delta * delta))) / 10 * (-1);
                    } else {
                        delta = (Math.sqrt(8 * delta - delta * delta)) / 10;
                    }
                    if (delta > 0.4) {
                        delta = 0.4;
                    } else if (delta < -0.4) {
                        delta = -0.4;
                    }
                    x = 0.1 + delta;
                    y = 0.1 - delta;
                }
                else {
                    if (delta > 0.2) {
                        delta = 0.2;
                    } else if (delta < -0.2) {
                        delta = -0.2;
                    }
                    x = -0.1 - delta;
                    y = -0.1 + delta;
                }


                if (lenth == 0 || lenth >= 150) {
                    robot.pushOnebyOne(0, 0);
                } else if (lenth == 24) {
                    robot.pushOnebyOne(0.6, 0.6);
                } else {
                    robot.pushOnebyOne(x, y);
                }
 //           }

    }
    public void test()
    {
        CaptureResult r;
        CaptureRequest q;
    }
    public void pushDeg(int deg,double x,double y){
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pushGamepad(x,y);
        while (opmode == null || opmode.opModeIsActive()) {
            if(Math.abs(robot.l2.getCurrentPosition()) >= deg) break;
        }
        robot.pushGamepad(0,0);
    }

}

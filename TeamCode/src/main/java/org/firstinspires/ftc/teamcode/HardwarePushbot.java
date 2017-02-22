package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  TMotor1     = null;
    public Servo wrench = null;
    public TouchSensor TCH =null;
    public double powerl = 0.0, powerr = 0.0;
    public DcMotor r1 = null, r2 = null;
    public DcMotor l1 = null, l2 = null;
    public DcMotor miniGun = null, collector = null;
    public OpticalDistanceSensor eye = null;
    public boolean isLoaded = false;
    //  public Servo    leftClaw    = null;
  //  public Servo    rightClaw   = null;

//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        l1 = hwMap.dcMotor.get("L1");
        l2 = hwMap.dcMotor.get("L2");
        r1 = hwMap.dcMotor.get("R1");
        r2 = hwMap.dcMotor.get("R2");
        wrench = hwMap.servo.get("wrench");
        collector = hwMap.dcMotor.get("collector");
        miniGun = hwMap.dcMotor.get("shooter");
        TCH = hwMap.touchSensor.get("TCH");
        eye = hwMap.opticalDistanceSensor.get("eye");
//        rightMotor  = hwMap.dcMotor.get("right_drive");
//        armMotor    = hwMap.dcMotor.get("left_arm");
//        TMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        l1.setPower(0);
        l2.setPower(0);
        r1.setPower(0);
        r2.setPower(0);
        miniGun.setPower(0);
        eye.enableLed(true);
        wrench.scaleRange(0.45,0.95);
        wrench.setPosition(1.0);
//        rightMotor.setPower(0);
//        armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        TMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        TMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        miniGun.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
//        leftClaw = hwMap.servo.get("left_hand");
//        rightClaw = hwMap.servo.get("right_hand");
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);
    }


    private void checkl()
    {
        if(powerl > 1.0)
        {
            powerr -= powerl - 1.0;
            powerl = 1.0;
        }
        if(powerl < -1.0)
        {
            powerr += -1.0 - powerl;
            powerl = 1.0;
        }
    }
    private void checkr()
    {
        if(powerr > 1.0)
        {
            powerl -= powerr - 1.0;
            powerr = 1.0;
        }
        if(powerr < -1.0)
        {
            powerl += -1.0 - powerr;
            powerr = 1.0;
        }
    }

    public void pushGamepad(double x, double y)
    {
        powerl = x-y;
        powerr = x+y;

        checkr();
        checkl();
/*
        if(powerl > 0)
        {
            l1.setDirection(DcMotor.Direction.REVERSE);
            l2.setDirection(DcMotor.Direction.REVERSE);
        }
        else
        {
            l1.setDirection(DcMotor.Direction.FORWARD);
            l2.setDirection(DcMotor.Direction.FORWARD);
        }
        if(powerr < 0)
        {
            r1.setDirection(DcMotor.Direction.REVERSE);
            r2.setDirection(DcMotor.Direction.REVERSE);
        }
        else
        {
            r1.setDirection(DcMotor.Direction.FORWARD);
            r2.setDirection(DcMotor.Direction.FORWARD);
        }
        */
        powerr *= -1;
        powerl *= -1;
        l1.setPower(powerl);
        l2.setPower(powerl);
        r1.setPower(powerr);
        r2.setPower(powerr);

    }
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period..
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    public void stop()
    {
        r1.setPower(0);
        r2.setPower(0);
        l1.setPower(0);
        l2.setPower(0);
        miniGun.setPower(0);
        collector.setPower(0);
        wrench.setPosition(1.0);//0.4
    }
}


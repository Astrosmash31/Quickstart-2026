package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.LookupTable;

public class SmartShooter {

    //Example declare a DcMotorEx object as part of this class called 'motorName'
    DcMotorEx motor;

    //Declare any other global variables for this class here
    private final LookupTable distanceToVelocity = new LookupTable(SmartShooterConstants.LOOKUP_TABLE);

    public SmartShooter(HardwareMap hardwareMap){
        this.motor = hardwareMap.get(DcMotorEx.class, SmartShooterConstants.MOTOR_NAME);

        //This defines the behavior at zero power (brake or coast)
        motor.setZeroPowerBehavior(SmartShooterConstants.ZERO_POWER_BEHAVIOR);

        //This defines the motor direction (forward or reversed)
        motor.setDirection(SmartShooterConstants.MOTOR_DIRECTION);

        /* This defines the motor velocity PIDF gains.  Velocity PIDF values determine control    *
         * around a target velocity (setTargetVelocity) OR how fast the system responds to a      *
         * change in set position (setTargetPosition).                                            */
        motor.setVelocityPIDFCoefficients(
                SmartShooterConstants.VELOCITY_P, //Proportional Gain
                SmartShooterConstants.VELOCITY_I, //Integral Gain
                SmartShooterConstants.VELOCITY_D, //Derivative Gain
                SmartShooterConstants.VELOCITY_F);//Feed Forward Gain

        /* This defines the motor position PID P gain. Position control only needs P gain since   *
         * once the system reaches the target position since once at position you're only         *
         * disturbances in the system                                                             */
        motor.setPositionPIDFCoefficients(
                SmartShooterConstants.POSITION_P);//Proportional Gain

        //motorName.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    /* Standard functions.  All Chelsea Robotics subsystems shall have init() and update() these  *
     * methods defined. Leave empty if not needed!                                                */
    public void init(){
        /* Call this method at the start of your opmode logic once to execute any logic you       *
         * want to be called on initialization. If none, leave empty!                             */
    }

    public void update(){
        //Call this method each time your opmode logic loops (i.e. inside while(opModeIsActive()){}
        //to execute any logic you want to be called periodically. If none, leave empty!

        //setTargetPosition needs to be called once per loop to keep the REV watchdog happy
        //motorName.setTargetPosition(motorSetPosition);
    }

    public void shoot(double distance) {
        double velocity = distanceToVelocity.interpolate(distance);
        this.setMotorVelocity(velocity);
        // TODO - is ball already engaged, or does it need to be dropped,
        // maybe after a short delay to allow the motor to spin up?
    }

    public void setMotorVelocity(double angularRate) {
        this.motor.setVelocity(angularRate);
    }
}

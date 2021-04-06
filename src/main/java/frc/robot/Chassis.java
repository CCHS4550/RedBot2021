package frc.robot;

import frc.parent.*;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Chassis implements RobotMap{

    //Talon objects for the wheels
    //These control the main 4 motors on the robot
    public static CCSparkMax fLeft = new CCSparkMax(RobotMap.FORWARD_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FORWARD_LEFT_REVERSE);
    public static CCSparkMax fRight = new CCSparkMax(RobotMap.FORWARD_RIGHT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FORWARD_RIGHT_REVERSE);
    public static CCSparkMax bLeft = new CCSparkMax(RobotMap.BACK_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_LEFT_REVERSE);
    public static CCSparkMax bRight = new CCSparkMax(RobotMap.BACK_RIGHT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_RIGHT_REVERSE);

    
    //AHRS gyro measures the angle of the bot
    public static AHRS gyro = new AHRS(SPI.Port.kMXP);

    //Solenoids
    public static Solenoid shiftOne = new Solenoid(RobotMap.SHIFT_SOLENOID_ONE);
    public static Solenoid shiftTwo = new Solenoid(RobotMap.SHIFT_SOLENOID_TWO);

    //Speed variables
    public static double lSpd = 0;
    public static double rSpd = 0; 

    //To be used in TeleOP
    //Takes in two axises, most likely the controller axises
    //Optimized for a west coast or standard chassis
    //DO NOT USE THIS FOR SWERV DRIVE 
    public static void axisDrive(double yAxis, double xAxis, double max){
        double spdMod = 0.005; //Accelerates to max in 4 seconds 
        double yIn = yAxis * RobotMap.ROBOT_Y_DIR_SIGN; //Joytick input
        double xIn = xAxis * RobotMap.ROBOT_X_DIR_SIGN;

        double lOut = OI.normalize((yIn - xIn), -max, max); //Joystick input into drivetrain input 
        double rOut = OI.normalize((yIn + xIn), -max, max);

        //Accelerates/Decelerates the left side via a ramp function 
        if(lOut > lSpd + spdMod)
            lSpd = lSpd + spdMod; 
        else 
            lSpd = rSpd - spdMod; 

        //Acceleraties/Decelerates the right side via a ramp function
        if(rOut > rSpd + spdMod)
            rSpd = rSpd + spdMod; 
        else 
            rSpd = rSpd - spdMod; 
    }

    //Zoom(Turns Solenoids on)
    public static void setFastMode(boolean on){
        shiftOne.set(!on);
        shiftTwo.set(on);
    }

    //To be used on Auto/PIDs
    //Simply sets the speeds to a certain percent output
    public static void driveSpd(double lSpeed, double rSpeed){
        lSpd = OI.normalize(lSpeed, -1.0, 1.0);
        rSpd = OI.normalize(rSpeed, -1.0, 1.0);
    }

    //Drives the robot 
    public static void drive(){
        fLeft.set(lSpd);
        fRight.set(rSpd);
        bLeft.set(lSpd);
        bRight.set(rSpd);
    }

    //Changes the converstion factor(for use with shifting gear boxes)
    public static void setFactor(double factor){
        //0.048 slow, 0.109 fast
        fLeft.setPositionConversionFactor(factor);
        fLeft.setPositionConversionFactor(factor);
        fLeft.setPositionConversionFactor(factor);
        fLeft.setPositionConversionFactor(factor);

    }

   
    //Sets the gyro and encoders to zero
    public static void reset(){
        gyro.reset();
        fLeft.reset();
        fRight.reset();
        bLeft.reset();
        bRight.reset();   

        lSpd = 0;
        rSpd = 0; 
    }

    public static double getLDist(){
        double dist = (fLeft.getPosition() + bLeft.getPosition())/2;
        return dist;
    }

    public static double getRDist(){
        double dist = (fRight.getPosition() + bRight.getPosition())/2;
        return dist;
    }

    public static double getAngle(){
        return gyro.getAngle();
    }

    /*
        "Whosever holds these loops, if he be worthy, shall posses the power of AJ"
    */

    //Drives the robot to a certain distance
    //Kinda complex -> DO NOT TOUCH
    public static void driveDist(double goal, double aPer, double kp, double max, boolean debug){
        setFactor(0.048);
        double aError = goal*aPer;

        double lPos = getLDist();
        double lError = goal-lPos;
        double lSpd = 0;

        double rPos = getRDist();
        double rError = goal-rPos;
        double rSpd = 0; 

        while(true){
            lPos = getLDist();
            lError = goal-lPos;
            lSpd = lError*kp;
            lSpd = OI.normalize(lSpd, -max, max);

            rPos = getRDist();
            rError = goal-rPos;
            rSpd = rError*kp;
            rSpd = OI.normalize(rSpd, -max, max);

            driveSpd(lSpd, rSpd);

            if(debug){
                System.out.println("Left - Left Speed: " + lSpd + 
                                        " Left Error: " + lError + 
                                        " Left Position: " + lPos);
                System.out.println("Right - Right Speed: " + rSpd + 
                                        " Right Error: " + rError + 
                                        " Right Position" + rPos);
                Timer.delay(0.5);
            }

            if(lError <= aError && rError <= aError){
                driveSpd(0.0, 0.0);
                System.out.println("YOINK, ya made it");
                break; 
            }
        }
    }

    //Turns the robot to a certain angle, a positive angle will turn right
    //Kinda complex -> DO NOT TOUCH
    public static void turnToAngle(double goal, double aPer, double kp, double max, boolean debug){
        double aError = goal*aPer;

        double angl = gyro.getAngle();
        double error = goal-angl;
        double input = 0;

        while(true){
            angl = gyro.getAngle();
            error = goal-angl;
            input = error*kp;
            input = OI.normalize(input, -max, max);

            driveSpd(input, -input);

            if(debug){
                System.out.println("Input: " + input);
                System.out.println("Error: " + error);
                System.out.println("Angle: " + angl);
                Timer.delay(0.5);
            }

            if(error <= aError){
                driveSpd(0.0, 0.0);
                System.out.println("YOINK, ya made it");
                break; 
            }
        }
    }


    
    
}
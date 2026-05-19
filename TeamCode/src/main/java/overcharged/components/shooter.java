package overcharged.components;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class shooter {

    RobotMecanum robot;
    private VoltageSensor chubVoltageSensor;

    public final OcMotorEx topShooter;
    public final OcMotorEx botShooter;

    public double targetSpeed;
    public double targetDist = 1.5;
    public double hood = Math.toRadians(57.628);
    public double lastError;
    public double integral;
    public double lastTime;

    public static double kP = 0.0235;
    public static double kI = 0.0007;
    public static double kD = 0.0000;
    public static double kV = 0.00005;
    public static double kS = 0.005;

    public static double targetVelocity;

    public float power = 0f;
    public float squid = 0f;
    public double POWERCOEFF = 1.86f;
    public double powerCoeff = POWERCOEFF; //ball initial speed, .265/(29-7/4.02-3.98)
    public double usedCoeff = POWERCOEFF;
    public float volPowerConstant;
    public float maxPow = 1.0f;
    private float curVolt;
    private float powerCoeffAdjuster = 0;
    private float secondaryAdjuster = 0;

    public double setTargetVel;

    public float flatPowerBooster = 0f;

    private double filteredVelocity = 0;
    private double alpha = 0.1;

    private double curVel;
    private double addCoeff;

    //TODO: motor velocity at max spin(1f) = 1.612
    //TODO:

    private double derivativePrev = 0;

    private boolean usePID = false;

    private boolean useSquid = false;

    public shooter(HardwareMap hardwareMap) {
        topShooter = new OcMotorEx(hardwareMap, "topShooter", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botShooter = new OcMotorEx(hardwareMap, "botShooter", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        chubVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void setPowerBoth(float power){
        topShooter.setPower(power);
        botShooter.setPower(power);
    }

    public void incPower() {
        powerCoeff = powerCoeff + 0.005;
    }

    public void decPower() {
        powerCoeff = powerCoeff - 0.005;
    }

    public void resetPower() {
        powerCoeff = POWERCOEFF;
    }

    public double getPowerCoeff() {
        return powerCoeff;
    }


    public void shoot(float power) {
        setPowerBoth(power);
    }
    public void shoot() {
        setPowerBoth(0.5714f);
    }

    public void intake() {
        setPowerBoth(-0.5f);
    }

    public void off() {
        setPowerBoth(0.01f);
    }

    public void setPower(float power)
    {
        int cnt = 1;
        //try {
        if (topShooter != null) {
            RobotLog.ii(TAG_SL, "Set slide left motor power to " + power);
            topShooter.setPower(power);
            if (power == 0f) {
                topShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorL");
        }
        cnt = 2;
        if (botShooter != null) {
            RobotLog.ii(TAG_SL, "Set slide right motor power to " + power);
            botShooter.setPower(power);
            if (power == 0f) {
                botShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorR");
        }
    }

    public float getPowerT() {
        return topShooter.getPower();
    }

    public float getPowerB() {
        return botShooter.getPower();
    }

    public float getPowerBoth() {
        return (Math.abs(botShooter.getPower())+Math.abs(topShooter.getPower()))/2;
    }

    public double getVelocityBoth() {return (Math.abs(botShooter.getVelocity()) + Math.abs(topShooter.getVelocity()))/2 ;}

    public static double ticksPerInch() {
        return (double) 2 * Math.PI / 28;
    }

    public float getTrueVel(double vel) {
        double rawVel = vel * 2 * Math.PI / 28 * 1.417;
        return (float) rawVel;
    }


    public float getTruePower(double vel) {
        double rawVel =  vel / ((2.0/3.0)*2*Math.PI/28*0.036);
        return (float) rawVel;
    }

    public void update() {
        if (usePID) {
            setPowerBoth(getSquid());
        }
//        if(useSquid) {
//            setPowerBoth(getSquidConstant(targetSpeed));
//        }

    }

    public float getVelocity() {
        //power outputs in velocity of shoot, finpower converts to motor power through vel of ball
        float power = (float) Math.sqrt((9.81 * Math.pow(targetDist, 2)) / (2 * Math.pow(Math.cos(hood), 2) * (targetDist * Math.tan(hood) - 0.70485)));
        if (targetDist < 1.10) {
            usedCoeff = powerCoeff - 0.13;
        } else if (targetDist < 1.15) {
            usedCoeff = powerCoeff - 0.11;
        } else if (targetDist < 1.200) {
            usedCoeff = powerCoeff - 0.09;
        } else if (targetDist < 1.35) {
            usedCoeff = powerCoeff - 0.07;
        } else if (targetDist < 1.7) {
            usedCoeff = powerCoeff - 0.05;
        } else if (targetDist > 2.6 && targetDist < 3.3) {
            usedCoeff = powerCoeff + 0.015;
        } else if (targetDist >= 3.3 && targetDist < 3.4) {
            usedCoeff = powerCoeff + 0.02;
        } else if (targetDist >= 3.4 && targetDist < 3.7) {
            usedCoeff = powerCoeff + 0.03;
        } else if (targetDist >= 3.7 && targetDist < 3.85) {
            usedCoeff = powerCoeff + 0.0375;
        } else if (targetDist >= 3.85) {
            usedCoeff = powerCoeff + 0.04;
        } else {
            usedCoeff = powerCoeff - 0.05;
        }
        return (float)usedCoeff*power*39.37f + volPowerConstant; // m/s to in/s
    }

//    public float getTrueVel(shooter motor) {
//        return motor.getVelocity()
//    }

    public float getSquid() {
        double error = getVelocity() + topShooter.getVelocity()*ticksPerInch();

        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        integral += error * dt;
        integral = Math.max(-100, Math.min(100, integral));

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double power = kP * error
                + kV * getVelocity()
                + kI * integral
                + kD * derivative
                + kS * Math.signum(error);

        return (float) Math.max(-1, Math.min(1, power));
    }

    public float getSquidConstant(double targetVel) {
        double error = getVelocity() - targetVel;
        double power = kP * error + kV * targetVelocity + kS * Math.signum(error);

        return (float) Math.max(-1, Math.min(1, power));
    }



    public double getError() {return lastError;}

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public void setUsePID(boolean usePID, double target, double hood) {
        this.usePID = usePID;
        this.targetDist = target/1000; //convert mm to m
        this.hood = hood;
    }

    public void setUseSquid(boolean useSquid) {
        this.useSquid = useSquid;
    }

    public void setUseSquid(boolean useSquid, int targetVel) {
        this.useSquid = useSquid;
        this.targetSpeed = targetVel;
    }

    public void setPVS(double kP, double kV, double kS) { this.kP = kP; this.kV = kV; this.kS = kS;}

    public void setTargetDist(int targetDist) { this.targetDist = targetDist;}

    public void setPowerCoeff(double powerCoeff) { this.powerCoeff = powerCoeff;}

    public void setFlatPowerBooster (float powerBoost) {this.flatPowerBooster = powerBoost;}


    public double getTargetSpeed() {
        return targetSpeed;
    }

}
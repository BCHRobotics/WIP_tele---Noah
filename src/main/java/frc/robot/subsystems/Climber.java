package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem{

    double ARMencoderCal = 1;
    double LEGencoderCal = 1;

    double encoderARM1last = 0;
    double encoderARM2last = 0;

    CANSparkMax ARM1 = new CANSparkMax(RobotMap.SPARKARM1, MotorType.kBrushless);
    CANSparkMax ARM2 = new CANSparkMax(RobotMap.SPARKARM2, MotorType.kBrushless);

    TalonSRX TALONLEG = new TalonSRX(RobotMap.TALONLEG);
    TalonSRX TALONLEGWHEEL = new TalonSRX(RobotMap.TALONLEGWHEEL);

    public CANEncoder encoderARM1 = new CANEncoder(ARM1);
    public CANEncoder encoderARM2 = new CANEncoder(ARM2);

    final int kTimeoutMs = 30;
    final boolean kDiscontinuityPresent = true;
	final int kBookEnd_0 = 910;		/* 80 deg */
	final int kBookEnd_1 = 1137;	/* 100 deg */

    public Climber(){

        TALONLEG.configFactoryDefault();

        initQuadrature();

        TALONLEG.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);

        ARM1.setInverted(true);
        ARM2.setInverted(false);

        TALONLEG.setInverted(false);
        TALONLEGWHEEL.setInverted(false);

    }

    public void armsMove(double speed){

        ARM1.set(speed);
        ARM2.set(speed);

    }

    public void legExtend(double speed){

        TALONLEG.set(ControlMode.PercentOutput, speed);

    }

    public void legWheel(double speed){

        TALONLEGWHEEL.set(ControlMode.PercentOutput, speed);

    }

    public double getEncoderARM1(){

        double encoderVal = -((encoderARM1.getPosition() * ARMencoderCal) - encoderARM1last);

        return encoderVal;
    }

    public double getEncoderARM2(){

        double encoderVal = -((encoderARM2.getPosition() * ARMencoderCal) - encoderARM2last);

        return encoderVal;
    }

    public double getEncoderAvg(){

        double result = (getEncoderARM1() + getEncoderARM2()) / 2;

        return result;
    }

    public double getEncoderLEG(){

        double encoderVal = TALONLEG.getSelectedSensorPosition(0);

        return encoderVal;
    }

    public void resetEncoderLEG(){
        TALONLEG.setSelectedSensorPosition(0, 0, this.kTimeoutMs);
    }

    public void resetEncoders(){

        encoderARM1last = encoderARM1.getPosition() * ARMencoderCal;
        encoderARM2last = encoderARM2.getPosition() * ARMencoderCal;
            
    }

    public void initQuadrature() {
		/* get the absolute pulse width position */
		int pulseWidth = TALONLEG.getSensorCollection().getPulseWidthPosition();

		/**
		 * If there is a discontinuity in our measured range, subtract one half
		 * rotation to remove it
		 */
		if (kDiscontinuityPresent) {

			/* Calculate the center */
			int newCenter;
			newCenter = (kBookEnd_0 + kBookEnd_1) / 2;
			newCenter &= 0xFFF;

			/**
			 * Apply the offset so the discontinuity is in the unused portion of
			 * the sensor
			 */
			pulseWidth -= newCenter;
		}

		/**
		 * Mask out the bottom 12 bits to normalize to [0,4095],
		 * or in other words, to stay within [0,360) degrees 
		 */
		pulseWidth = pulseWidth & 0xFFF;

		/* Update Quadrature position */
		TALONLEG.getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);
    }
    
    @Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		//setDefaultCommand(new DriveArcade());
	}

}
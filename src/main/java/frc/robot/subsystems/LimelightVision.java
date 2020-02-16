package frc.robot.subsystems;

import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

/**
 *
 */
public class LimelightVision extends PIDSubsystem {
	
	private NetworkTable table;
	
	private Target target;
	
    public LimelightVision() {
		//super(new PIDController(0.05, 0, 0.01));
		//super(new PIDController(0.05, 0, 0.0025));
		super(new PIDController(0.05, 0, 0.0040));
    	table = NetworkTableInstance.getDefault().getTable("limelight");
    	
    	//horizontalAlignPid = new PIDController(0.05, 0, 0.01);
    	//horizontalAlignPid = new PIDController(0.015, 0.001, 0.01, source, output);
		setSetpoint(0);
		getController().setTolerance(Constants.POSITION_TOLERANCE, Constants.VELOCITY_TOLERANCE);
	}

	public void updateLimelightData() {
		NetworkTableEntry tv = table.getEntry("tv");
		boolean targetExists = tv.getDouble(0) > 0.0;
	//	System.out.println("Tv:" + tv.getDouble(0));
		
		if (!targetExists) {
			disable();
			target = null;
			return;
		}
		
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");

		/*
		NetworkTableEntry tx0 = table.getEntry("tx0");
		NetworkTableEntry ty0 = table.getEntry("ty0");
		NetworkTableEntry ta0 = table.getEntry("ta0");
		*/
		
		target = new Target();
		target.x = tx.getDouble(0);
		target.y = ty.getDouble(0);
		target.a = ta.getDouble(0);
		
		if (!isEnabled()) enable();
	}

	public void updateShuffleBoard() {
		SmartDashboard.putBoolean("haveTarget", target != null);
		if (target != null) {
			SmartDashboard.putNumber("targetX", target.x);
			SmartDashboard.putNumber("targetY", target.y);
			SmartDashboard.putNumber("targetA", target.a);
		}
		//System.out.println("Target a:" + ta);
	}

    public void initDefaultCommand() {
    }
    
    /**
     * Get most recent target parameters
     * @return a Target object with target parameters, otherwise null if no target is found
     */
    public Target getTargetValues() {
    	return target;
    }
	
	public boolean getTargetV() {
		NetworkTableEntry tv = table.getEntry("tv");
		System.out.println(tv.getDouble(0));
		if (tv.getDouble(0) > 0.0) {
			return true;
		} else {
			return false;
		}
	}

	public double getTargetArea(){
		return table.getEntry("ta").getDouble(0);
	}

	public double getTargetX() {
		return table.getEntry("tx").getDouble(0);
	}
    
    public static class Target {
    	public double x;
    	public double y;
		public double a;
		
		public double x0;
    	public double y0;
    	public double a0;
    }
    
    public void turnOffLight() {
    	NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");
    	ledMode.setNumber(1);
    }
    
    public void turnOnLight() {
    	NetworkTableEntry ledMode = table.getEntry("ledMode");
    	ledMode.setNumber(0);
    }
    
    public void flushNetworkTables() {
		NetworkTableInstance.getDefault().flush();
	}
    
    public void setPipeline(int pipeline) {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
		pipelineEntry.setNumber(pipeline);
		pipelineEntry.setDefaultNumber(pipeline);
		pipelineEntry.forceSetNumber(pipeline);
	}
    
    public void setCamMode(int camMode) {
		NetworkTableEntry camModeEntry = table.getEntry("camMode");
		camModeEntry.setNumber(camMode);

	}

	public double findDistance(double h1, double h2, double a1, double a2)  {
		return ((h2-h1) / (Math.tan(Math.toRadians(a1+a2))));
	}

	public void turnWithLimelight(DoubleConsumer useOutput, boolean end) {
		setPipeline(8);
		if(end) {
			useOutput.accept(0);
			setPipeline(0);
		} else {
			useOutput.accept(getController().calculate(getTargetX(), getController().getSetpoint()));
		}
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		
	}

	@Override
	protected double getMeasurement() {
		return target.x;
	}
}

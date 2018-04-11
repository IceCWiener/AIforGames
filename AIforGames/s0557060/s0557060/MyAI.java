package s0557060;
import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DriverAction;
import lenz.htw.ai4g.ai.Info;

public class MyAI extends AI {
	
	float x;
	float y;
	float ori;
	float checkP;
	float track;
	
	public MyAI(Info info) {
		super(info);
	}

	@Override
	public String getName() {
		return "Konstantim";
	}

	@Override
	public DriverAction update(boolean wasResetAfterCollision) {
		
		x = info.getX();	//Startpunkt = (500, 500)
		y = info.getY();
		info.getAngularVelocity();
		ori = info.getOrientation();	//-3.14 bis 3.14
		info.getVelocity();
		info.getCurrentCheckpoint();
		info.getTrack();
		
		//Consoleprints der Werte
		//System.out.println("x: " + x + ", y: " + y);
		//System.out.println("Orientation: " + ori);
		//System.out.println("Current Checkpoint: " + checkP);
		//System.out.println("Track: " + track);
		
		return new DriverAction(1, 1);
	}
	
	@Override
	public String getTextureResourceName() {
		return "/s0557060/car.png";
	}
	
	@Override
	public boolean isEnabledForRacing() {
		return true;
	}
	
	

}

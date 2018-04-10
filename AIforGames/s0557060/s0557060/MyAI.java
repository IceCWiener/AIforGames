package s0557060;
import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DriverAction;
import lenz.htw.ai4g.ai.Info;

public class MyAI extends AI {
	
	public MyAI(Info info) {
		super(info);
	}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return "Konstantim";
	}

	@Override
	public DriverAction update(boolean wasResetAfterCollision) {
		
		info.getX();
		info.getY();
		info.getAngularVelocity();
		info.getOrientation();
		info.getVelocity();
		info.getCurrentCheckpoint();
		info.getTrack();
		
		return new DriverAction(10, 10);
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

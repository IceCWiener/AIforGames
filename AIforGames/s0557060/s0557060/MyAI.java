package s0557060;
import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DriverAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.track.Track;
import java.lang.Math;

public class MyAI extends AI {
	
	float x;
	float y;
	float ori;
	float checkP;
	Track track;
	float richtung;
	int i = 0;
	
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
		track = info.getTrack();
		
		//Consoleprints der Werte
		//System.out.println("x: " + x + ", y: " + y);
		//System.out.println("Orientation: " + ori);
		//System.out.println("Current Checkpoint: " + checkP);
		//System.out.println("Track: " + track);
		
		richtung = lenkung(x, y, 900, 100);
		System.out.println(harmReihe());
		return new DriverAction(1, harmReihe());
	}
	
	@Override
	public String getTextureResourceName() {
		return "/s0557060/car.png";
	}
	
	@Override
	public boolean isEnabledForRacing() {
		return true;
	}
	
	private int lenkung(float ax, float ay, float bx, float by) {
		int richtung = 0;
		//Skalarprodukt
//		double skala = ax*bx + ay*by;
//		//Satz des Pythagoras
//		//float a2 = ax - bx;
//		//float b2 = ay - by;
//		
//		double distA = Math.sqrt(Math.pow(ax, 2) + Math.pow(ay, 2));
//		double distB = Math.sqrt(Math.pow(bx, 2) + Math.pow(by, 2));
//		double degWink = Math.acos(skala/(distA*distB));
//		double piWink = Math.toRadians(degWink);
		float deltaX = ax - bx;		
		float deltaY = ay - by;
		float angle = (float) Math.atan2(deltaY,  deltaX);
		float wink = (float) Math.toRadians(angle);
//		System.out.println("Skala: " + skala);
//		System.out.println("Wink: " + wink);
		//System.out.println("deltaX: " + deltaX);
		//System.out.println("deltaY: " + deltaY);
		//System.out.println("Winkel zwischen den Punkten: " + angle);
		

		if (ori > 0.785 + 0.2) {
			richtung = -1;
			return richtung;
		} else if(ori < 0.785 - 0.2){
			richtung = 1;
			return richtung;
		}else {
			richtung = 0;
			return richtung;
		}
	}
	
	public int harmReihe () {
		int richtung;
		i = i+1;
		if(i%3 == 0) {
			return -1;
		}else {
			return 1;
		}
			//i = i+1;
			//richtung = (int) Math.pow(-1, i);
			//System.out.println(richtung);
			//return richtung;
	}

}

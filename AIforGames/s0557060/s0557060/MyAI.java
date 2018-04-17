package s0557060;
import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DriverAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.track.Track;
import lenz.htw.ai4g.*;
import java.lang.Math;
import org.lwjgl.opengl.GL11;

import java.awt.Polygon;

public class MyAI extends AI {
	
	float x;
	float y;
	float ori;
	float checkP;
	Track track;
	float richtung;
	int i = 0;
	Polygon[] obstacles = info.getTrack().getObstacles(); //Hindernisse vom Typ Polygon
	int obsX;
	int obsY;
	int autoX;
	int autoY;
	
	public MyAI(Info info) {
		super(info);
		//nlistForDevelopment(); //Nur zum testen
		enlistForTournament(557060, 556736); //Beide Matrikelnummern übergeben für Anmeldung als Gruppe
		
		//int ziel1 = obstacles[2].npoints; //Zeigt Punkte des Polygons
		
//		obstacles[2].xpoints[3]; //Koordinaten des dritten Punktes
//		obstacles[2].ypoints[3];
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
		obsX = obstacles[0].xpoints[0];
		obsY = obstacles[0].ypoints[0];
		autoX = (int) info.getX();
		autoY = (int) info.getY();
		
		//Consoleprints der Werte
		//System.out.println("x: " + x + ", y: " + y);
		System.out.println("Orientation: " + ori);
		//System.out.println("Current Checkpoint: " + checkP);
		//System.out.println("Track: " + track);
		richtung = lenkung(x, y, obsX, obsY);
		//System.out.println(harmReihe());
		return new DriverAction(1, richtung);
	}
	
	@Override
	public String getTextureResourceName() {
		return "/s0557060/car.png";
	}
	
//	@Override
//	public boolean isEnabledForRacing() {
//		return true;
//	}
	
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
		float punkWi = urschpWink(obsX, obsY);
		float autoWi = urschpWink(autoX, autoY);
		
		

		if (ori > 0.785 + 0.2) { //0.785pi = ca 45°
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
	
	public float urschpWink(int x, int y) {
		//float wink;
		float piWink;
		float m;
		m = y / x;
		piWink = (float) Math.atan(m);
		return piWink;		
	}
	
	public void doDebugStuff() {
		int obsX = obstacles[0].xpoints[0];
		int obsY = obstacles[0].ypoints[0];
		
		GL11.glBegin(GL11.GL_LINES);
		GL11.glVertex2f(info.getX(), info.getY());
		GL11.glVertex2d(info.getX() + Math.cos(info.getOrientation()) * 15, info.getY() + Math.sin(info.getOrientation()) * 15);
		GL11.glEnd();
		GL11.glBegin(GL11.GL_LINES);
		
		GL11.glVertex2f(info.getX(), info.getY());
		GL11.glVertex2f(obsX, obsY);
		//GL11.glVertex2f(0, 0);
		GL11.glEnd();
	}

}

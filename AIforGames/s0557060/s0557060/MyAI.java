package s0557060;
import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DriverAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.track.Track;
import lenz.htw.ai4g.*;
import java.lang.Math;
import org.lwjgl.opengl.GL11;
import org.lwjgl.util.vector.Vector2f;

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
	float obsX;
	float obsY;
	int autoX;
	int autoY;
	float zielgeschwindigkeit;
	float startgeschwindigkeit;
	float drehBeschleunigung;
	float wunschDrehGeschw;
	float maxTurnSpeed;
	float abbremsWinkel = (float)(Math.PI);	//Winkel ab dem das Auto bremsen soll, um nicht zu übersteuern.
	float turnSpeed;
	float maxAcceleration = info.getMaxAcceleration();
	float wunschZeit = 1;	//Genaue Funktion noch unklar
	Vector2f getVelCoord;
	float getVel;
	float maxVel;
	float abbremsRad = 5;
	float rotZielAbsWinkel;	//Winkel zwischen der Orientiereung des Autos und dem Ziel
	float zielWinkel;
	float rotZielWinkel;
	float toleranz = 0.4f;
	float lenkSpeed;
	
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
		System.out.println("AngularVelo: " + info.getAngularVelocity());
		ori = info.getOrientation();	//-3.14 bis 3.14
		getVelCoord = info.getVelocity();
		getVel = (float) Math.sqrt(Math.pow(getVelCoord.x, 2) + Math.pow(getVelCoord.y, 2));	//Länge des Velocity-Vektors (Geschwindigkeit)
		//System.out.println("Velocity: " + getVel);
		System.out.println("Checkpoint Charlie: " + info.getCurrentCheckpoint());
		track = info.getTrack();
		obsX = (float) info.getCurrentCheckpoint().getX();
		obsY = (float) info.getCurrentCheckpoint().getY();
		autoX = (int) info.getX();
		autoY = (int) info.getY();
		maxVel = info.getMaxVelocity();	//Maximale Geschwindigkeit ist 28.0
		maxTurnSpeed = info.getMaxAngularVelocity(); //1.5
		turnSpeed = Math.abs(info.getAngularVelocity());
		//System.out.println("maxAnglAcce: " + info.getMaxAngularAcceleration());
		oriWink(); //Errechnet den Winkel zwischen den Orientierungen
		
		//Consoleprints der Werte
		//System.out.println("x: " + x + ", y: " + y);
		System.out.println("Orientation: " + ori);
		//System.out.println("Current Checkpoint: " + checkP);
		//System.out.println("Track: " + track);
		richtung = lenkung(abbremsWinkel);
		//System.out.println(harmReihe());
		//System.out.println("Beschleunigung: " + drehBeschleunigung);
		System.out.println("Winkel zw Orientierungen(Betrag): " + rotZielAbsWinkel);
		//System.out.println("Winkel zw Orientierungen: " + rotZielWinkel);
		
		
		return new DriverAction(beschleunigung(), lenkung(abbremsWinkel));
	}

	private void oriWink() {
		zielWinkel = atan2Vector(x, y, obsX, obsY);
		rotZielWinkel = zielWinkel - ori;
		if(rotZielWinkel >= Math.PI) {
			rotZielWinkel = (float) (-Math.PI + (rotZielWinkel - Math.PI));
		} else if(rotZielWinkel <= -Math.PI) {
			rotZielWinkel = (float) (Math.PI + (rotZielWinkel + Math.PI));
		}
		rotZielAbsWinkel = Math.abs(rotZielWinkel);
	}
	
	@Override
	public String getTextureResourceName() {
		return "/s0557060/car.png";
	}
	
//	@Override
//	public boolean isEnabledForRacing() {
//		return true;
//	}
	
	private float beschleunigung() {		//Translation
		float ret;
		float bremsPunktX = 0;
		float bremsPunktY = 0;
		float wunschGeschw = 0;
		
		if(abstand(x, y, obsX, obsY) <= abbremsRad + 1 && abstand(x, y, obsX, obsY) > abbremsRad - 1) {
			bremsPunktX = x;
			bremsPunktY = y;
		}
		
		if(abstand(x, y, obsX, obsY) < abbremsRad) {	//Arrive
			wunschGeschw = abstand(x, y, obsX, obsY) / abstand(bremsPunktX, bremsPunktY, obsX, obsY) * maxVel / abbremsRad;			
		} else {
			if(rotZielAbsWinkel < toleranz) {
				wunschGeschw = maxVel;	//TODO: Geschwindigkeit in Abhängigkeit auf Richtung
//			} else if(rotZielAbsWinkel < abbremsWinkel) {	//Align
//				wunschDrehGeschw = rotZielAbsWinkel * (maxTurnSpeed / abbremsWinkel);
//				if(wunschDrehGeschw > maxTurnSpeed) {	//Clippingabfrage
//					wunschDrehGeschw = maxTurnSpeed;
//				}
//				wunschGeschw = maxVel * (wunschDrehGeschw / maxTurnSpeed);
			} else {
//				wunschDrehGeschw = maxTurnSpeed;
//				wunschGeschw = maxVel * (wunschDrehGeschw / maxTurnSpeed);
				wunschGeschw = -(maxVel/maxTurnSpeed) * wunschDrehGeschw + maxVel;
			}
		}
		ret = (wunschGeschw - getVel) / wunschZeit;
		System.out.println("WunschgeschW: " + wunschGeschw);
		return ret;
	}
	
	private float lenkung(float abbremsWinkel) {	//Rotation
		float richtung;
		
		if(rotZielWinkel < 0) {
			abbremsWinkel = 0 - abbremsWinkel;
		} else {
			abbremsWinkel = this.abbremsWinkel;
		}
		if(rotZielWinkel < abbremsWinkel && abbremsWinkel > 0) {
			richtung = rotZielWinkel * (maxTurnSpeed / abbremsWinkel);
		} else if(rotZielWinkel > abbremsWinkel && abbremsWinkel < 0){
			richtung = rotZielWinkel * (maxTurnSpeed / abbremsWinkel);
		} else if(rotZielWinkel < 0){
			richtung = 1;
		} else if(rotZielWinkel > 0) {
			richtung = -1;
		} else {
			richtung = 0;
		}
		
//		float zielWinkel = atan2Vector(ax, ay, bx,by);	//Winkel des Vektors vom Auto zum Ziel (Ursprung ist Auto)
//		System.out.println("zielWinkel: " + zielWinkel); 
//		
//		float rotZielWinkel = zielWinkel - ori;	//Winkel zwischen der Orientiereung des Autos und dem Ziel
//		System.out.println("rotZielWinkel: " + rotZielWinkel);
//		
//		if(rotZielWinkel < abbremsWinkel) {
//			wunschDrehGeschw = Math.abs(rotZielWinkel * maxTurnSpeed / abbremsWinkel);
//		} else {
//			wunschDrehGeschw = maxTurnSpeed;
//		}
//		System.out.println("Wunschdreh: " + wunschDrehGeschw);
//		
//		drehBeschleunigung = idealBeschleunigung(turnSpeed, wunschDrehGeschw, wunschZeit); //Zwischen 0 und 1.5 
////		
//		if(drehBeschleunigung > maxAcceleration) {	//Clippingabfrage
//			drehBeschleunigung = maxAcceleration;
//		} //TODO: Min Clipping (?)
//		
//		richtung = (float) (rotZielWinkel/Math.PI);
//		if(rotZielWinkel <= Math.PI/4) {
//			richtung = -1;
//		}else if(rotZielWinkel >= -Math.PI/4) {
//			richtung = 1;
//		}
//		if(rotZielWinkel <= Math.PI/4 && turnSpeed < 0.5 && rotZielWinkel >= -Math.PI/4) {
//			richtung = 0;
//		}
		return richtung;
//		if (ori > rotZielWinkel) { 
//			richtung = -1;
//			return richtung;
//		} else if(ori < rotZielWinkel){
//			richtung = 1;
//			return richtung;
//		}else {
//			richtung = 0;
//			return richtung;
//		}
	}
	
//	public int harmReihe () {
//		int richtung;
//		i = i+1;
//		if(i%3 == 0) {
//			return -1;
//		}else {
//			return 1;
//		}
//			//i = i+1;
//			//richtung = (int) Math.pow(-1, i);
//			//System.out.println(richtung);
//			//return richtung;
//	}
	
//	public float urschpWink(int x, int y) {
//		//float wink;
//		float piWink;
//		float m;
//		m = y / x;
//		piWink = (float) Math.atan(m);
//		return piWink;		
//	}
	
	public void doDebugStuff() {
		
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(255, 0, 0);
		GL11.glVertex2f(info.getX(), info.getY());
		GL11.glVertex2d(info.getX() + Math.cos(info.getOrientation()) * 15, info.getY() + Math.sin(info.getOrientation()) * 15);
		GL11.glEnd();
		
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(0, 255, 0);
		GL11.glVertex2f(info.getX(), info.getY());
		GL11.glVertex2f(obsX, obsY);
		//GL11.glVertex2f(0, 0);
		GL11.glEnd();
		
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(0, 0, 255);
		GL11.glVertex2f(x,  y);
		GL11.glVertex2f(x + getVelCoord.x, y + getVelCoord.y);
		GL11.glEnd();
	}
	
	public float atan2Vector(double ax, double ay, double bx, double by) {
		float zielCoordY;
		float zielCoordlX;
		float steigungsWinkel;
		float steigungM;
		
//		if(ax > bx) {
//			zielCoordY = (float)(ay-by); 
//			zielCoordlX = (float)(ax-bx);
//		} else {
			zielCoordY = (float)(by-ay); 
			zielCoordlX = (float)(bx-ax);
		//}
		
		//steigungM = zielCoordY / zielCoordlX;
		steigungsWinkel = (float) Math.atan2(zielCoordY, zielCoordlX);
		
		return steigungsWinkel;
	}
	
	public float idealBeschleunigung(float start, float ziel, float wunschZeit) {
		float beschleunigung;
		
		beschleunigung = (ziel - start) / wunschZeit;
				
		return beschleunigung;
	}
	
	public float abstand (float startX, float startY, float zielX, float zielY) {
		float ret;
		float ergX;
		float ergY;
		
		ergX = zielX - startX;
		ergY = zielY - startY;
		
		ret =(float) Math.sqrt(Math.pow(ergX, 2)+ Math.pow(ergY, 2));
		return ret;
	}

}
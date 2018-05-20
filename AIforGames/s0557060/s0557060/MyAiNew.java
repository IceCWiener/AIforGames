package s0557060;

import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DriverAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.track.Track;
import lenz.htw.ai4g.*;
import java.lang.Math;
import org.lwjgl.opengl.GL11;
import org.lwjgl.util.Point;
import org.lwjgl.util.vector.Vector2f;

import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.*;

public class MyAiNew extends AI {

	float x;
	float y;
	float nX;
	float nY;
	float ori;
	float checkP;
	Track track;
	float richtung;
	int i = 0;
	Polygon[] obstacles = info.getTrack().getObstacles(); // Hindernisse vom Typ Polygon
	Rectangle bounds;		
	
	float obsX;
	float obsY;
	float zielgeschwindigkeit;
	float startgeschwindigkeit;
	float drehBeschleunigung;
	float wunschDrehGeschw;
	float maxTurnSpeed;
	float abbremsWinkel = (float) (Math.PI/2); // Winkel ab dem das Auto bremsen
	
	float bremsWinkel = 0.3f;
	float bremsRadius = 30;
	float beschlGes;											// soll, um nicht zu
	float drehBeschlGes;
	float wunschGeschw;
																// übersteuern.
	float turnSpeed;
	float maxAcceleration = info.getMaxAcceleration();
	float wunschZeit = 1; // Genaue Funktion noch unklar
	Vector2f getVelCoord;
	float getVel;
	float maxVel;
	float abbremsRad = 10;
	float stopRad = 5;
	float stopRadWP = 10;
	float stopRadWP2 = 2;
	float rotZielAbsWinkel; // Winkel zwischen der Orientiereung des Autos und
							// dem Ziel
	float zielWinkel;
	float rotZielWinkel;
	float toleranz = 0.01f;
	float lenkSpeed;
	float maxAngularAccelaration = info.getMaxAngularAcceleration();;
	
	double gruenM;
	double rotM;
	double gruenL;
	double rotL;
	double gruenR;
	double rotR;
	double sensorLinksX;
	double sensorLinksY;
	double sensorMitteX;
	double sensorMitteY;
	double sensorRechtsX;
	double sensorRechtsY;
	int sensLaenge = 16;
	int sensLaengeSeite = 8;
	boolean hit = false;
	boolean wp1 = true;
	boolean wp2 = true;
	boolean wp3 = false;	
	int zielQ = 1;
	Point pWegPunkt = new Point();
	Point pAnfahrt = new Point();

	public MyAiNew(Info info) {
		super(info);
		// nlistForDevelopment(); //Nur zum testen
		enlistForTournament(557060, 556736); // Beide Matrikelnummern übergeben
												// für Anmeldung als Gruppe

		// int ziel1 = obstacles[2].npoints; //Zeigt Punkte des Polygons

		// obstacles[2].xpoints[3]; //Koordinaten des dritten Punktes
		// obstacles[2].ypoints[3];
	}

	@Override
	public String getName() {
		return "KonstantimNEW";
	}

	@Override
	public DriverAction update(boolean wasResetAfterCollision) {

		x = info.getX(); // Startpunkt = (500, 500)
		y = info.getY();
		nX = (float) (Math.cos(ori));
		nY = (float) (Math.sin(ori));
		ori = info.getOrientation(); // -3.14 bis 3.14
		getVelCoord = info.getVelocity();
		getVel = (float) Math.sqrt(Math.pow(getVelCoord.x, 2) + Math.pow(getVelCoord.y, 2)); // Länge
																								// des
																								// Velocity-Vektors
																								// (Geschwindigkeit)
		// System.out.println("Velocity: " + getVel);
		// System.out.println("Checkpoint Charlie: " +
		// info.getCurrentCheckpoint());
		track = info.getTrack();
		obsX = (float) info.getCurrentCheckpoint().getX();
		obsY = (float) info.getCurrentCheckpoint().getY();
//		obsX = 600;
//		obsY = 600;
		
		//Kollision
		Rectangle2D sensL = new Rectangle2D.Double(sensorLinksX-1, sensorLinksY-1, 2, 2);
		Rectangle2D sensR = new Rectangle2D.Double(sensorRechtsX-1, sensorRechtsY-1, 2, 2);
		Rectangle2D sensM = new Rectangle2D.Double(sensorMitteX-1, sensorMitteY-1, 2, 2);
		
		for(Polygon p : obstacles) {
			if(p.contains(sensL)) {
				rotL = 255;
				gruenL= 0;
				hit = true;
				//System.out.println("Linker Sensor aktiviert!");
			} else {
				rotL = 0;
				gruenL = 255;
			}
			if(p.contains(sensR)) {
				rotR = 255;
				gruenR = 0;
				hit = true;
				//System.out.println("Rechter Sensor aktiviert!");
			} else {
				rotR = 0;
				gruenR = 255;
			}
			if(p.contains(sensM)) {
				rotM = 255;
				gruenM = 0;
				hit = true;
				//System.out.println("Mittlerer Sensor aktiviert!");
			} else {
				rotM = 0;
				gruenM = 255;
			}		
		}
		//Wegpunkte
		if(hit == true) {
			Point pZiel = new Point(info.getCurrentCheckpoint().x, info.getCurrentCheckpoint().y);
			int autoQ = quadTr11(x, y);
			zielQ = quadTr11(pZiel.getX(), pZiel.getY());
			Point p1 = new Point(375, 625);
			Point p2 = new Point(625, 625);
			Point p3 = new Point(375, 375);
			Point p4 = new Point(625, 375);
			
			float zielWeg = abstand(x, y, pWegPunkt.getX(), pWegPunkt.getY());
			float umWeg = abstand(x, y, pAnfahrt.getX(), pAnfahrt.getY());
			
			while (autoQ == 1){ 
				if(wp2) {
					
					pAnfahrt = p1;
					wp2 = false;
					break;
				}
				if(zielQ == 1) {
					wp2 = true;
					wp3 = false;
					break;
				}else if(zielQ == 2) {					
					if(umWeg < stopRadWP) {
						pWegPunkt = p2;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 3) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p3;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 4) {
					if(umWeg < stopRadWP) {
						wp3 = true;
						pWegPunkt = p2;
					}
					break;
				}
			}while (autoQ == 2){ 
				if(wp2) {
					pAnfahrt = p2;
					wp2 = false;
					break;
				}
				if(zielQ == 1) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p1;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 2) {	
					wp2 = true;	
					wp3 = false;	
					break;
				}else if(zielQ == 3) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p1;
						wp3 = true;
					}
					break;
				}else if(zielQ == 4) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p4;
						wp2 = true;
						wp3 = false;
					}
					break;
				}
			}while (autoQ == 3){
				if(wp2) {
					pAnfahrt = p3;
					wp2 = false;
					break;
				}
				if(zielQ == 1) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p1;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 2) {					
					if(umWeg < stopRadWP) {
						pWegPunkt = p1;
						wp3 = true;
					}
					break;
				}else if(zielQ == 3) {
					wp2 = true;
					break;
				}else if(zielQ == 4) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p4;
						wp2 = true;
						wp3 = false;
					}
					break;
				}
			}while(autoQ == 4){
				if(wp2) {
					pAnfahrt = p4;
					wp2 = false;
					break;
				}
				if(zielQ == 1) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p2;
						wp3 = true;
					}
					break;
				}else if(zielQ == 2) {					
					if(umWeg < stopRadWP) {
						pWegPunkt = p2;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 3) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p3;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 4)
					wp2 = true;
					wp3 = false;
					break;
				}
			System.out.println("AutoQ : " + autoQ);
			System.out.println("zielQ :" + zielQ);
			System.out.println("Ziel X: " + obsX + "|| Ziel Y: " + obsY);
			zielWeg = abstand(x, y, pWegPunkt.getX(), pWegPunkt.getY());
			umWeg = abstand(x, y, pAnfahrt.getX(), pAnfahrt.getY());
			//System.out.println("Zielweg: " + zielWeg);
			
			if (wp1) {
				obsX = pAnfahrt.getX();
				obsY = pAnfahrt.getY();
				System.out.println("1. Punkt");
			}
			
			if (umWeg < stopRadWP) {
				obsX = pWegPunkt.getX();
				obsY = pWegPunkt.getY();
				wp1 = false;
				System.out.println("2. Punkt");
			}
			
			if(zielWeg < stopRadWP && !wp3 && !wp2) {
				obsX = pZiel.getX();
				obsY = pZiel.getY();
				hit = false;
				wp1 = true;
				wp2 = true;
			}
			//System.out.println("dualWp: " + dualWP);
		}
		
		maxVel = info.getMaxVelocity(); // Maximale Geschwindigkeit ist 28.0
		maxTurnSpeed = info.getMaxAngularVelocity(); // 1.5
		turnSpeed = info.getAngularVelocity();
		// System.out.println("maxAnglAcce: " +
		// info.getMaxAngularAcceleration());
		oriWink(); // Errechnet den Winkel zwischen den Orientierungen
		collisionDetection();
		//driveCalc();
		
		// Consoleprints der Werte
		// System.out.println("x: " + x + ", y: " + y);
		// System.out.println("Orientation: " + ori);
		// System.out.println("Current Checkpoint: " + checkP);
		// System.out.println("Track: " + track);
		//richtung = lenkung(abbremsWinkel);
		// toleranz = emerTol(getVel);
		// System.out.println(harmReihe());
		// System.out.println("Beschleunigung: " + drehBeschleunigung);
		// System.out.println("Winkel zw Orientierungen(Betrag): " +
		// rotZielAbsWinkel);
		//System.out.println("Winkel zw Orientierungen: " + rotZielWinkel);
		//System.out.println("Richtung: " + lenkung(abbremsWinkel));
		//System.out.println("Toleranz: " + toleranz);
//		System.out.println("Beschleunigung: " + beschleunigung());
//		System.out.println("Drehbeschleunigung: " + lenkung());
//		System.out.println("TurnSpeed: " + info.getAngularVelocity());
		
		return new DriverAction(beschleunigung(), lenkung());
	}
	
	private int quadTr11(float x, float y) {	//Unterteilt Track 10 in vier Quadranten und erkennt in welchem der Punkt des Parameters gerade ist.
		int quad;
		if(x < 500 && y > 500) {
			quad = 1;
		} else if(x > 500 && y > 500) {
			quad = 2;
		} else if(x < 500 && y < 500) {
			quad = 3;
		}else if(x > 500 && y < 500) {
			quad = 4;
		} else {
			quad = 0;
		}
		return quad;
	}

	private void oriWink() {
		zielWinkel = atan2Vector(x, y, obsX, obsY);
		rotZielWinkel = zielWinkel - ori;
		if (rotZielWinkel >= Math.PI) {
			rotZielWinkel = (float) (rotZielWinkel - 2*Math.PI);
		} else if (rotZielWinkel <= -Math.PI) {
			rotZielWinkel = (float) (rotZielWinkel + 2*Math.PI);
		}
		rotZielAbsWinkel = Math.abs(rotZielWinkel);
	}

	@Override
	public String getTextureResourceName() {
		return "/s0557060/car.png";
	}
	
	
//	public void driveCalc(){
//		
//	
//	float zielWeg = abstand(x,y,obsX,obsY);
//		
//		if(rotZielAbsWinkel < toleranz){
//			wunschDrehGeschw = 0;
//			wunschGeschw = maxVel;
//		}else if(zielWeg < stopRad){
//			wunschGeschw = 0;
//		}
//		
//		if(rotZielAbsWinkel < bremsWinkel && zielWeg > bremsRadius){
//			wunschDrehGeschw = rotZielWinkel * maxTurnSpeed / bremsWinkel;
//			wunschGeschw = maxVel;
//		} else if(rotZielAbsWinkel > bremsWinkel){
//			wunschGeschw = 1;
//			if(rotZielWinkel >= 0){
//				wunschDrehGeschw = maxTurnSpeed;
//			}else{
//				wunschDrehGeschw = -maxTurnSpeed;
//			}
//		}else if(zielWeg < bremsRadius){
//			wunschGeschw = zielWeg * maxVel / bremsRadius;
//		}
//		
//		wunschDrehGeschw = wunschDrehGeschw / maxTurnSpeed;
//		
//		boolean hit = false;
//		
//		if(hit){
//			wunschGeschw = -info.getMaxBackwardVelocity();
//			wunschDrehGeschw = -maxTurnSpeed;
//		}
//		
//		beschlGes = (wunschGeschw - getVelCoord.length()) / 1;
//		
//		if(beschlGes > maxAcceleration){
//			beschlGes = maxAcceleration;
//		}
//		
//		drehBeschlGes = (wunschDrehGeschw - turnSpeed) / 1;
//		
//		if(drehBeschlGes > maxAngularAccelaration){
//			drehBeschlGes = maxAngularAccelaration;
//		}
		
//	}
	
//	public boolean collision(int obsNum, Line2D sens){
//		Point p = new Point((int) info.getX() + Math.cos(info.getOrientation()) * 15,(int) info.getX() + Math.cos(info.getOrientation()) * 15);
//	}
	// @Override
	// public boolean isEnabledForRacing() {
	// return true;
	// }

	private float beschleunigung() { // Translation
		float ret; 
		float wunschGeschw = 0;
		float zielWeg = abstand(x,y,obsX,obsY);

		if(rotZielAbsWinkel < toleranz){
			wunschGeschw = maxVel;
		}else if(zielWeg < stopRad){
			wunschGeschw = 0;
		}
		
		if(rotZielAbsWinkel < bremsWinkel && zielWeg > bremsRadius){
			wunschGeschw = maxVel;
		} else if(rotZielAbsWinkel > bremsWinkel){
			wunschGeschw = 1;
		}else if(zielWeg < bremsRadius){
			wunschGeschw = zielWeg * maxVel / bremsRadius;
		}
		
		ret = (wunschGeschw - getVel) / wunschZeit;
		
		if(ret > maxAcceleration){
			ret = maxAcceleration;
		}
		
		return ret;
	}

	private float lenkung() { // Rotation
		float richtung;
		float wunschDrehGeschw = 0;
		float zielWeg = abstand(x,y,obsX,obsY);
		
		if(rotZielAbsWinkel < toleranz){
			wunschDrehGeschw = 0;
		}
		
		if(rotZielAbsWinkel < bremsWinkel && zielWeg > bremsRadius){
			wunschDrehGeschw = rotZielWinkel * maxTurnSpeed / bremsWinkel;
		} else if(rotZielWinkel >= 0){
				wunschDrehGeschw = maxTurnSpeed;
		}else{
				wunschDrehGeschw = -maxTurnSpeed;
		}
		
		wunschDrehGeschw = wunschDrehGeschw / maxTurnSpeed;
		
		richtung = (wunschDrehGeschw - turnSpeed) / wunschZeit;
		
		if(richtung > maxAngularAccelaration){
			richtung = maxAngularAccelaration;
		}

		// if(rotZielWinkel < 0) {
		// abbremsWinkel = 0 - abbremsWinkel;
		// } else {
		// abbremsWinkel = this.abbremsWinkel;
		// }
		// if(rotZielWinkel < abbremsWinkel && abbremsWinkel > 0) {
		// richtung = rotZielWinkel * (maxTurnSpeed / abbremsWinkel);
		// System.out.println("Abbwink > 0: " + abbremsWinkel);
		// } else if(rotZielWinkel > abbremsWinkel && abbremsWinkel < 0){
		// richtung = rotZielWinkel * (maxTurnSpeed / abbremsWinkel);
		// System.out.println("Abbwink < 0: " + abbremsWinkel);

		// DAS
		// HIER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// Berechnung der Drehgeschwindigkeit mit Winkeln im Betrag(werden unten
		// + oder -)
//		if (rotZielAbsWinkel < toleranz) {
//			wunschDrehGeschw = 0;
//			System.out.println("Genau richtig!");
//		} else if (rotZielAbsWinkel < abbremsWinkel) {
//			wunschDrehGeschw = rotZielWinkel * maxTurnSpeed / abbremsWinkel;
//			System.out.println("lenk ein bisschen!");
//		} else {
//			if(rotZielWinkel > 0){
//				wunschDrehGeschw = maxTurnSpeed;
//				System.out.println("Lenk doll links!");
//			} else {
//				wunschDrehGeschw = -maxTurnSpeed;
//				System.out.println("Lenk doll rechts!");
//			}
//		}
//		//wunschDrehGeschw = wunschDrehGeschw/maxTurnSpeed;
//		// turnSpeed = -(maxTurnSpeed/1) * richtung + maxTurnSpeed;
//		richtung = (wunschDrehGeschw - turnSpeed) / wunschZeit;
//		if (richtung > 1) {
//			richtung = 1f; // MaxDrehBeschleunigung
//		} else if(richtung < -1){
//			richtung = -1;
//		}
		
		

//		if (rotZielWinkel > 0 + toleranz) {
//			return richtung = richtung;
//		} else if (rotZielWinkel < 0 - toleranz) {
//			return richtung = -richtung;
//		}

		// keine Benutzung der Winkel im Betrag:
		// if(rotZielAbsWinkel < abbremsWinkel) {
		// if(rotZielWinkel <= 0) {
		// wunschDrehGeschw = rotZielWinkel * (maxTurnSpeed / abbremsWinkel);
		// } else {
		// wunschDrehGeschw = rotZielWinkel * (maxTurnSpeed / -abbremsWinkel);
		// }
		// } else if(rotZielWinkel < 0 && rotZielAbsWinkel > abbremsWinkel) {
		// wunschDrehGeschw = -2.5f;
		// if(wunschDrehGeschw < -1) {
		// wunschDrehGeschw = -1;
		// }
		// } else if(rotZielWinkel > 0 && rotZielAbsWinkel > abbremsWinkel){
		// wunschDrehGeschw = 2.5f;
		// if(wunschDrehGeschw > 1) {
		// wunschDrehGeschw = 1;
		// }

		// richtung = wunschDrehGeschw - turnSpeed / wunschZeit;

		// float zielWinkel = atan2Vector(ax, ay, bx,by); //Winkel des Vektors
		// vom Auto zum Ziel (Ursprung ist Auto)
		// System.out.println("zielWinkel: " + zielWinkel);
		//
		// float rotZielWinkel = zielWinkel - ori; //Winkel zwischen der
		// Orientiereung des Autos und dem Ziel
		// System.out.println("rotZielWinkel: " + rotZielWinkel);
		//
		// if(rotZielWinkel < abbremsWinkel) {
		// wunschDrehGeschw = Math.abs(rotZielWinkel * maxTurnSpeed /
		// abbremsWinkel);
		// } else {
		// wunschDrehGeschw = maxTurnSpeed;
		// }
		// System.out.println("Wunschdreh: " + wunschDrehGeschw);
		//
		// drehBeschleunigung = idealBeschleunigung(turnSpeed, wunschDrehGeschw,
		// wunschZeit); //Zwischen 0 und 1.5
		////
		// if(drehBeschleunigung > maxAcceleration) { //Clippingabfrage
		// drehBeschleunigung = maxAcceleration;
		// } 
		//
		// richtung = (float) (rotZielWinkel / Math.PI);
		// if (rotZielWinkel <= Math.PI / 4) {
		// richtung = -1;
		// } else if (rotZielWinkel >= -Math.PI / 4) {
		// richtung = 1;
		// }
		// if (rotZielWinkel <= Math.PI / 4 && turnSpeed < 0.5 && rotZielWinkel
		// >= -Math.PI / 4) {
		// richtung = 0;
		// }
		return richtung;
		// if (ori > rotZielWinkel) {
		// richtung = -1;
		// return richtung;
		// } else if(ori < rotZielWinkel){
		// richtung = 1;
		// return richtung;
		// }else {
		// richtung = 0;
		// return richtung;
		// }
	}

	// public int harmReihe () {
	// int richtung;
	// i = i+1;
	// if(i%3 == 0) {
	// return -1;
	// }else {
	// return 1;
	// }
	// //i = i+1;
	// //richtung = (int) Math.pow(-1, i);
	// //System.out.println(richtung);
	// //return richtung;
	// }

	// public float urschpWink(int x, int y) {
	// //float wink;
	// float piWink;
	// float m;
	// m = y / x;
	// piWink = (float) Math.atan(m);
	// return piWink;
	// }
//	
//	private boolean collision(Rectangle2D col) {
//		return bounds.intersects(col);
//	}
//	
	public void collisionDetection() {
		
//		Rectangle2D sensL = new Rectangle2D.Double(sensorLinksX-1, sensorLinksY-1, 2, 2);
//		Rectangle2D sensR = new Rectangle2D.Double(sensorRechtsX-1, sensorRechtsY-1, 2, 2);
//		Rectangle2D sensM = new Rectangle2D.Double(sensorMitteX-1, sensorMitteY-1, 2, 2);
//		
//		for(Polygon p : obstacles) {
//			if(p.contains(sensL)) {
//				rotL = 255;
//				gruenL= 0;
//				System.out.println("Linker Sensor aktiviert!");
//				obsX = 500;
//				obsY = 100;
//			} else {
//				rotL = 0;
//				gruenL = 255;
//			}
//			if(p.contains(sensR)) {
//				rotR = 255;
//				gruenR = 0;
//				System.out.println("Rechter Sensor aktiviert!");
//				obsX = 500;
//				obsY = 100;
//			} else {
//				rotR = 0;
//				gruenR = 255;
//			}
//			if(p.contains(sensM)) {
//				rotM = 255;
//				gruenM = 0;
//				System.out.println("Mittlerer Sensor aktiviert!");
//				obsX = 500;
//				obsY = 100;
//			} else {
//				rotM = 0;
//				gruenM = 255;
//			}		
//		}
		
	}
	
	public void doDebugStuff() {

		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(255, 0, 0);
		GL11.glVertex2f(info.getX(), info.getY());
		GL11.glVertex2d(info.getX() + Math.cos(info.getOrientation()) * 15,	info.getY() + Math.sin(info.getOrientation()) * 15);
		GL11.glEnd();

		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(0, 255, 0);
		GL11.glVertex2f(info.getX(), info.getY());
		GL11.glVertex2f(obsX, obsY);
		GL11.glEnd();

		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(0, 0, 255);
		GL11.glVertex2f(x, y);
		GL11.glVertex2f(x + getVelCoord.x, y + getVelCoord.y);
		GL11.glEnd();
		
		//Darstellung der Kollisionssensoren
		sensorMitteX = (x + Math.cos(ori)*sensLaenge);
		sensorMitteY = (y + Math.sin(ori)*sensLaenge);
		sensorLinksX = (x + Math.cos(ori + Math.PI/3)*sensLaengeSeite);
		sensorLinksY = (y + Math.sin(ori + Math.PI/3)*sensLaengeSeite);
		sensorRechtsX = (x + Math.cos(ori - Math.PI/3)*sensLaengeSeite);
		sensorRechtsY = (y + Math.sin(ori - Math.PI/3)*sensLaengeSeite);

		GL11.glBegin(GL11.GL_POINTS);
		GL11.glColor3d(rotM, gruenM, 0);
		GL11.glVertex2d(sensorMitteX, sensorMitteY); //Mitte
		GL11.glVertex2d(sensorLinksX, sensorLinksY); //Links
		GL11.glVertex2d(sensorRechtsX, sensorRechtsY); //Rechts
		GL11.glEnd();
	}

	public float atan2Vector(double ax, double ay, double bx, double by) {
		float zielCoordY;
		float zielCoordlX;
		float steigungsWinkel;

		// if(ax > bx) {
		// zielCoordY = (float)(ay-by);
		// zielCoordlX = (float)(ax-bx);
		// } else {
		zielCoordY = (float) (by - ay);
		zielCoordlX = (float) (bx - ax);
		// }

		// steigungM = zielCoordY / zielCoordlX;
		steigungsWinkel = (float) Math.atan2(zielCoordY, zielCoordlX);

		return steigungsWinkel;
	}

	public float idealBeschleunigung(float start, float ziel, float wunschZeit) {
		float beschleunigung;

		beschleunigung = (ziel - start) / wunschZeit;

		return beschleunigung;
	}

	public float abstand(float startX, float startY, float zielX, float zielY) {
		float ret;
		float ergX;
		float ergY;

		ergX = zielX - startX;
		ergY = zielY - startY;

		ret = (float) Math.sqrt(Math.pow(ergX, 2) + Math.pow(ergY, 2));
		return ret;
	}

	public float emerTol(float vel) { // erweiterte Toleranz im Falle eines
										// Notfalls
		if (vel < 12) {
			return 1.3f;
		} else if (vel < 20) {
			return 0.2f;
		} else {
			return 0.1f;
		}
	}

}
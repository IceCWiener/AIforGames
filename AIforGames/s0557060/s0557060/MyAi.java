package s0557060;

import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DriverAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.track.Track;
import lenz.htw.ai4g.*;
import java.lang.Math;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import org.lwjgl.opengl.GL11;
import org.lwjgl.util.Point;
import org.lwjgl.util.vector.Vector2f;

import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.geom.*;

public class MyAi extends AI{

	float x = info.getX();
	float y = info.getY();
	float nX;
	float nY;
	float ori;
	float checkPX;
	float checkPY;
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
	
	float bremsWinkel = 0.6f;
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
	float bremsGeschw = 5;
	float stopRad = 0;
	float stopRadWP = 10;
	float stopRadWP2 = 2;
	float rotZielAbsWinkel; // Winkel zwischen der Orientiereung des Autos und
							// dem Ziel
	float rotZielAbsWinkelCP;
	float zielWinkel;
	float zielWinkelCP;
	float zielAbsWinkelCP;
	float rotZielWinkel;
	float rotZielWinkelCP;
	float toleranz = 0.01f;
	float lenkSpeed;
	float maxAngularAccelaration = info.getMaxAngularAcceleration();;
	
	double gruenM;
	double rotM;
	double gruenL;
	double rotL;
	double gruenR;
	double rotR;
	float sensorLinksX;
	float sensorLinksY;
	float sensorMitteX;
	float sensorMitteY;
	float sensorRechtsX;
	float sensorRechtsY;
	int sensLaenge = 16;
	int sensLaengeSeite = 8;
	boolean hit = false;
	boolean wp1 = true;
	boolean wp2 = true;
	boolean wp3 = false;	
	int zielQ = 1;
	Point pWegPunkt = new Point();
	Point pAnfahrt = new Point();
	//info.getFastZones(); //info.getSlowZones() //Für Tracks 30+
	float width;
	float height;
	
	//Rastervariablen
	static int cellSize = 20;
	Boolean arrB[][];
	int w;
	int h;
	boolean raster = false;
	//Node Generator
	Node nodes[][];
	int startPx;
	int startPy;
	boolean startPunkt = false;
	boolean neuerWegpunkt = false;
	//List<Integer> pathInts = new ArrayList<Integer>(Arrays.asList(25, 30, 26, 30, 27, 30, 27, 31, 27, 32, 26, 32, 25, 32, 25, 31, 26, 32, 27, 33, 28, 34, 29, 35, 29, 36, 29, 37));
	List<Integer> pathInts = new ArrayList<Integer>();
	int iHit = 0;
	int jHit = 1;
	int[] points;
	
	public MyAi(Info info) {
		super(info);
		// nlistForDevelopment(); //Nur zum testen
		enlistForTournament(557060, 556736);
		
		track = info.getTrack();
		width = track.getWidth();
		height = track.getHeight();
		maxVel = info.getMaxVelocity(); // Maximale Geschwindigkeit ist 28.0
		maxTurnSpeed = info.getMaxAngularVelocity(); // 1.5
		arrB = new Boolean[(int) width/cellSize][(int) height/cellSize];
		nodes = new Node[(int) width/cellSize][(int) height/cellSize];
		rastern();
	}

	@Override
	public String getName() {
		return " ";
	}

	@Override
	public DriverAction update(boolean wasResetAfterCollision) {

		x = info.getX(); // Startpunkt = (500, 500)
		y = info.getY();
		if(!startPunkt) {
			startPx = (int)(x/cellSize);
			startPy = (int)(y/cellSize);	
			startPunkt = true;
		}
		nX = (float) (Math.cos(ori));
		nY = (float) (Math.sin(ori));
		ori = info.getOrientation(); // -3.14 bis 3.14
		getVelCoord = info.getVelocity();
		getVel = (float) Math.sqrt(Math.pow(getVelCoord.x, 2) + Math.pow(getVelCoord.y, 2)); 
		turnSpeed = info.getAngularVelocity();

		collisionDetect();	//Sensoren erkennen Berührung
		//wegpunktMethode();	//Auswahl der richtigen Wegfindungsmethode
		//System.out.println(neuerWegpunkt);
		if(!neuerWegpunkt) {
			checkPX = info.getCurrentCheckpoint().x;
			checkPY = info.getCurrentCheckpoint().y;
			nodesAStarSearchAndPrint();
			neuerWegpunkt = true;
			//System.out.println(neuerWegpunkt);
		}
		followLikeNSubscribe();	//Setzt obsX&Y auf currentCheckpoint
		oriWink(); // Errechnet den Winkel zwischen den Orientierungen
		//oriWinkCP();
		prints();	//Alle Sysouts
		
		return new DriverAction(beschleunigung(), lenkung());
	}
    public void nodeGenerator(int i, int j){
    		
    		String name = Integer.toString(i) + " " + Integer.toString(j);
    		nodes[i][j] = new Node(name, abstand(i, j, checkPX, checkPY));
    }
    
    public int frameCheck(int x) {
    	// TODO width/height unterscheiden!(?)
    	if(x < 0) {
    		return 0;
    	}else if(x >= nodes.length) {
    		return (int) nodes.length-1;
    	}
    	return x;
    }

	public void addEdges() {
		float ab = cellSize;
		float abDia = (float) Math.floor(abstand(0, 0, cellSize, cellSize));
		int kernelSize = 1;
		int counter = 0;
		int edgeNum = 4;
		ArrayList<Edge> edges;
		
		for(int i = 0; i < nodes.length; i++) {
			for(int j = 0; j < nodes.length; j++) {
				
				nodes[i][j].adjacencies = new ArrayList<Edge>();
				
				if (arrB[i][j]) {
					for (int k = -kernelSize; k <= kernelSize; k++) {
						for (int h = -kernelSize; h <= kernelSize; h++) {
							if (counter == edgeNum) {
								counter = 0;
							}
							if (k == 0 ^ h == 0) {
								if (i + k < 0 || j + h < 0 || i + k > nodes.length - 1 || j + h > nodes.length - 1) {
									continue;
								} else if (arrB[i + k][j + h]) {
									if (arrB[i + k][j + h] == null) {
										continue;
									}
									nodes[i][j].adjacencies.add(new Edge(nodes[i + k][j + h], cellSize));
									//System.out.println(" Source: " + nodes[i][j].nodeX + "-" + nodes[i][j].nodeY + " Edgetarget: " + nodes[i][j].adjacencies[counter].targetX + "-" + nodes[i][j].adjacencies[counter].targetY);
									//System.out.println("Source: " + nodes[i][j].nodeX + "-" + nodes[i][j].nodeY + " Target: " + nodes[i+k][j+h].nodeX + "-" + nodes[i+k][j+h].nodeY);
									counter++;
								}
							}
						}
					} 
				}
				edges = nodes[i][j].getEdgeArr();
				if(nodes[i][j].adjacencies == null || edges == null) {
					continue;
				}
//				for(Edge e : edges){
//					System.out.println("Source: " + nodes[i][j].nodeX + "-" + nodes[i][j].nodeY + " Target: " + e.targetX + "-" + e.targetY);
//				}
			}
		}
	}

	public List<Node> printPath(Node target) {
		List<Node> path = new ArrayList<Node>();

		for (Node node = target; node != null; node = node.parent) {
			path.add(node);
		}

		Collections.reverse(path);
		for (Node e : path) {
			intPathConverter(e);
		}
		points = new int[pathInts.size()];
		//System.out.println("PathSize:" + pathInts.size());
		for (int j = 0; j < pathInts.size(); j++) {
			points[j] = pathInts.get(j);
			//System.out.print(", " + points[j]);
		}
		return path;
	}

    public void intPathConverter(Node node) {
    	String name = node.value;
    	String[] parts = name.split(" ");
    	String part1 = parts[0];
    	String part2 = parts[1];
    	int part1Int = Integer.parseInt(part1);
    	int part2Int = Integer.parseInt(part2);
    	pathInts.add(part1Int);
    	pathInts.add(part2Int);
    }
    
	public void AstarSearch(Node source, Node goal) {

		ArrayList<Edge> edges;
		
		Set<Node> explored = new HashSet<Node>();

		PriorityQueue<Node> queue = new PriorityQueue<Node>(arrB.length, new Comparator<Node>() {
			// override compare method
			public int compare(Node i, Node j) {
				return (int) Math.signum(j.f_scores - i.f_scores);
			}

		});

		// cost from start
		source.g_scores = 0;

		queue.add(source);

		boolean found = false;

		//System.out.print("Explored: ");
		while ((!queue.isEmpty()) && (!found)) {

			// the node in having the lowest f_score value
			Node current = queue.poll();
			edges = current.getEdgeArr();

			explored.add(current);
			//System.out.print(" |" + current + "| ");

			// goal found
			if (current.value.equals(goal.value)) {
				found = true;
			}

			// check every child of current node
			for (Edge e : edges) {
				Node child = e.target;
				
				e.cost = e.cost - abstand(e.targetX, e.targetY, checkPX, checkPY);
				
				double temp_g_scores = current.g_scores + e.cost;
				double temp_f_scores = temp_g_scores + child.h_scores;

				/*
				 * if child node has been evaluated and the newer f_score is higher, skip
				 */

				if ((explored.contains(child))) {
					continue;
				}

				/*
				 * else if child node is not in queue or newer f_score is lower
				 */

				else if ((!queue.contains(child)) || (temp_f_scores < child.f_scores)) {

					child.parent = current;
					child.g_scores = temp_g_scores;
					child.f_scores = temp_f_scores;

					if (!queue.contains(child)) {
						queue.add(child);
					}
					//System.out.println("Child: " + child);

				}

			}

		}
		// System.out.println();
		// System.out.println("queue:");
		// for(Node n : queue) {
		// System.out.print(n);
		// }
		// System.out.println("explored:");
		// for(Node n : explored) {
		// System.out.print(n);
		// }
	}

	public void rastern() {		
		for(int i = 0; i < arrB.length; i++) {
			for(int j = 0; j < arrB[i].length; j++) {
				Rectangle2D r = new Rectangle(i*cellSize, j*cellSize, cellSize, cellSize);
				arrB[i][j] = true;
				for (int k = 0; k < obstacles.length; k++) {
					if (obstacles[k].intersects(r)) {
						arrB[i][j] = false;
						break;
					}
				}
				System.out.print(arrB[i][j]?".":"+");
			}
			System.out.println();
		}
	}

	public void nodesAStarSearchAndPrint() {
		int endPx = (int)(checkPX/cellSize);
		int endPy = (int)(checkPY/cellSize);
		for(int i = 0; i < arrB.length; i++) {
			for(int j = 0; j < arrB.length; j++) {
				nodeGenerator(i, j);
			}
		}
        addEdges();
		AstarSearch(nodes[startPx][startPy], nodes[endPx][endPy]);	//Pfad nimmt die Startzelle des Autos sowie die Zielzelle in der sich der Checkpoint befindet. 
		//TODO Neue Startzelle für das Auto berechnen nachdem getCurrentCheckpoint() sich ändert.

        List<Node> path = printPath(nodes[endPx][endPy]);	//node[endPx][endPy]	//Test: [47][47]
        System.out.println();
        System.out.println("Path: " + path);
        //System.out.println("selbstgeschriebener Weg" + pathInts);
	}
	
	public void followLikeNSubscribe() {
		if(hit == false) {
			double tempX = 0;
			double tempY = 0;
			
			obsX = points[iHit] * cellSize + cellSize/2;
			obsY = points[jHit] * cellSize + cellSize/2;
			if(abstand( x, y, obsX, obsY) < 15 && jHit < pathInts.size()-2) {
					iHit += 2;
					jHit += 2;
			}
			
			if(abstand(x, y, pathInts.get(pathInts.size()-2)*cellSize + cellSize/2, pathInts.get(pathInts.size()-1)*cellSize+cellSize/2) < 15) {
				obsX = (float) info.getCurrentCheckpoint().getX();
				obsY = (float) info.getCurrentCheckpoint().getY();
			}
			
			if (abstand(x, y, (float) info.getCurrentCheckpoint().getX(), (float) info.getCurrentCheckpoint().getY()) <= 2) {
				neuerWegpunkt = false;
			}
		}
	}

	public void wegpunktMethode() {
		int counter = 0;
		for(Polygon p : obstacles) {
			counter++;
		}
		
		switch (counter) {
		case 2:
			
			break;
		case 3:
			wegpunkteTr11();
			break;
		case 4:
			wegpunkteTr12();
			break;
		case 6: 
			if(!raster) {
				rastern();
				raster = true;
			}
			if(!neuerWegpunkt) {
				nodesAStarSearchAndPrint();
				neuerWegpunkt = true;
			}
			break;	
		case 8:
			if(!raster) {	//Raster wird einmalig erstellt
				rastern();
				raster = true;
			}
			if(!neuerWegpunkt) {
				nodesAStarSearchAndPrint();	//A* wird nach jedem neuem Checkpoint neu ausgeführt
				neuerWegpunkt = true;
			}
			break;
			
		default:
			break;
		}
	}

	public void prints() {
		// Consoleprints der Werte
		// System.out.println("maxAnglAcce: " + info.getMaxAngularAcceleration());
		// System.out.println("Velocity: " + getVel);
		// System.out.println("Checkpoint Charlie: " + info.getCurrentCheckpoint());
		//System.out.println("Breite" + width);	//Tr12: 1000	//Tr11: 1000
//		System.out.println("Höhe" + height);	//Tr12: 800		//Tr11: 1000
		// System.out.println("x: " + x + ", y: " + y);
		// System.out.println("Orientation: " + ori);
		 //System.out.println("Current Checkpoint: " + info.getCurrentCheckpoint().getX() + " " + info.getCurrentCheckpoint().getY());
		// System.out.println("Track: " + track);
		//richtung = lenkung(abbremsWinkel);
		// toleranz = emerTol(getVel);
//		 System.out.println(harmReihe());
		// System.out.println("Beschleunigung: " + drehBeschleunigung);
		//System.out.println("Winkel zw Orientierungen(Betrag): " + rotZielAbsWinkelCP);
		//System.out.println("Winkel zw Orientierungen: " + rotZielWinkel);
		//System.out.println("Richtung: " + lenkung(abbremsWinkel));
		//System.out.println("Toleranz: " + toleranz);
//		System.out.println("Beschleunigung: " + beschleunigung());
//		System.out.println("Drehbeschleunigung: " + lenkung());
//		System.out.println("TurnSpeed: " + info.getAngularVelocity());
//		System.out.println("arrb.length: " + arrB.length);
//		System.out.println("iHit: " + iHit + " jHit: " + jHit);
//		System.out.println("obsX: " + obsX + " obsY: " + obsY);
		//System.out.println("The Path: " + pathInts);
		//System.out.println("Zielwinkel: " + zielWinkel);
	}

	public void collisionDetect() {
		
		sensorMitteX = (float) (x + Math.cos(ori)*sensLaenge);
		sensorMitteY = (float)(y + Math.sin(ori)*sensLaenge);
		sensorLinksX = (float)(x + Math.cos(ori + Math.PI/3)*sensLaengeSeite);
		sensorLinksY = (float)(y + Math.sin(ori + Math.PI/3)*sensLaengeSeite);
		sensorRechtsX = (float)(x + Math.cos(ori - Math.PI/3)*sensLaengeSeite);
		sensorRechtsY = (float)(y + Math.sin(ori - Math.PI/3)*sensLaengeSeite);
		
		//Kollision
//		Rectangle2D sensL = new Rectangle2D.Double(sensorLinksX-1, sensorLinksY-1, 2, 2);
//		Rectangle2D sensR = new Rectangle2D.Double(sensorRechtsX-1, sensorRechtsY-1, 2, 2);
//		Rectangle2D sensM = new Rectangle2D.Double(sensorMitteX-1, sensorMitteY-1, 2, 2);
		
		for(Polygon p : obstacles) {
			if(p.contains(sensorLinksX, sensorLinksY)) {
				rotL = 255;
				gruenL= 0;
				hit = true;
				//System.out.println("Linker Sensor aktiviert!");
			} else {
				rotL = 0;
				gruenL = 255;
			}
			if(p.contains(sensorRechtsX, sensorRechtsY)) {
				rotR = 255;
				gruenR = 0;
				hit = true;
				//System.out.println("Rechter Sensor aktiviert!");
			} else {
				rotR = 0;
				gruenR = 255;
			}
			if(p.contains(sensorMitteX, sensorMitteY)) {
				rotM = 255;
				gruenM = 0;
				hit = true;
				//System.out.println("Mittlerer Sensor aktiviert!");
			} else {
				rotM = 0;
				gruenM = 255;
			}		
		}
	}

	private void wegpunkteTr11() {
		//Wegpunkte
		if(hit == true) {
			Point pZiel = new Point(info.getCurrentCheckpoint().x, info.getCurrentCheckpoint().y);
			int autoQ = quadTr11(x, y);
			zielQ = quadTr11(pZiel.getX(), pZiel.getY());
			Point p1 = new Point(275, 625);
			Point p2 = new Point(725, 625);
			Point p3 = new Point(275, 375);
			Point p4 = new Point(725, 375);
			float zielWeg = abstand(x, y, pWegPunkt.getX(), pWegPunkt.getY());
			float umWeg = abstand(x, y, pAnfahrt.getX(), pAnfahrt.getY());
			
			switch(autoQ) {
			case 1: 
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
			case 2: 
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
			case 3: 
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
			case 4: 
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
				}else if(zielQ == 4){
					wp2 = true;
					wp3 = false;
					break;
				}
			}
			//System.out.println("AutoQ : " + autoQ);
			//System.out.println("zielQ :" + zielQ);
			System.out.println("WP1: " + wp1 + "|| WP2 : " + wp2 + "|| WP3: " + wp3);
			//System.out.println("pAnfahrt: " + pAnfahrt);
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
				wp3 = false;
				System.out.println("2. Punkt");
			}
			
			if(zielWeg < stopRadWP && !wp3 && !wp2) {
				System.out.println("3. Punkt");
				obsX = pZiel.getX();
				obsY = pZiel.getY();
				hit = false;
				wp1 = true;
				wp2 = true;
			}
			//System.out.println("dualWp: " + dualWP);
		}
	}

	private void wegpunkteTr12() {
		//Wegpunkte
		int counter = 0;
		if(counter > 2) {
			hit = false;
		}
		if(hit == true) {
			Point pZiel = new Point(info.getCurrentCheckpoint().x, info.getCurrentCheckpoint().y);
			int autoQ = quadTr12(x, y);
			zielQ = quadTr12(pZiel.getX(), pZiel.getY());
			Point p1 = new Point((int) (1./6.*width), (int) (3./4.*height));
			Point p2 = new Point((int) (3./6.*width), (int) (3./4.*height));
			Point p3 = new Point((int) (5./6.*width), (int) (3./4.*height));
			Point p4 = new Point((int) (1./6.*width), (int) (1./4.*height));
			Point p5 = new Point((int) (3./6.*width), (int) (1./4.*height));
			Point p6 = new Point((int) (5./6.*width), (int) (1./4.*height));
			
			float zielWeg = abstand(x, y, pWegPunkt.getX(), pWegPunkt.getY());
			float umWeg = abstand(x, y, pAnfahrt.getX(), pAnfahrt.getY());
			
			switch(autoQ) {
			case 1: 
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
						pWegPunkt = p4;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 5) {
					if(umWeg < stopRadWP) {
						wp3 = true; 
						pWegPunkt = p4;
					}
					break;
				}else if(zielQ == 6) {
					if(umWeg < stopRadWP) {
						wp3 = true;
						pWegPunkt = p4;
					}
					break;
				}
			case 2: 
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
						pWegPunkt = p3;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 4) {
					if(umWeg < stopRadWP) {
						wp3 = true;
						pWegPunkt = p5;
					}
					break;
				}else if(zielQ == 5) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p5;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 6) {
					if(umWeg < stopRadWP) {
						wp3 = true;
						pWegPunkt = p5;
					}
					break;
				}
			case 3:
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
						pWegPunkt = p2;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 3) {
					wp2 = true;
					wp3 = false;
					break;
				}else if(zielQ == 4) {
					if(umWeg < stopRadWP) {
						wp3 = true;
						pWegPunkt = p6;
					}
					break;
				}else if(zielQ == 5) {
					if(umWeg < stopRadWP) {
						wp3 = true;
						pWegPunkt = p6;
					}
					break;
				}else if(zielQ == 6) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p6;
						wp2 = true;
						wp3 = false;
					}
					break;
				}
			case 4:
				if(wp2) {
					pAnfahrt = p4;
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
					wp2 = true;
					wp3 = false;
					break;
				}else if(zielQ == 5) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p5;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 6) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p6;
						wp2 = true;
						wp3 = false;
					}
					break;
				}
			case 5:
				if(wp2) {
					pAnfahrt = p5;
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
						pWegPunkt = p2;
						wp2 = true;
						wp3 = false;
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
				}else if(zielQ == 5) {
					wp2 = true;
					wp3 = false;
					break;
				}else if(zielQ == 6) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p6;
						wp2 = true;
						wp3 = false;
					}
					break;
				}
			case 6:
				if(wp2) {
					pAnfahrt = p6;
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
				}else if(zielQ == 4) {
					pWegPunkt = p4;
					wp2 = true;
					wp3 = false;
					break;
				}else if(zielQ == 5) {
					if(umWeg < stopRadWP) {
						pWegPunkt = p5;
						wp2 = true;
						wp3 = false;
					}
					break;
				}else if(zielQ == 6) {
					wp2 = true;
					wp3 = false;
					break;
				}
			}
//			System.out.println("AutoQ : " + autoQ);
//			System.out.println("zielQ :" + zielQ);
//			System.out.println("Ziel X: " + obsX + "|| Ziel Y: " + obsY);
			zielWeg = abstand(x, y, pWegPunkt.getX(), pWegPunkt.getY());
			umWeg = abstand(x, y, pAnfahrt.getX(), pAnfahrt.getY());
			//System.out.println("Zielweg: " + zielWeg);
			
			if (wp1) {
				obsX = pAnfahrt.getX();
				obsY = pAnfahrt.getY();
//				System.out.println("1. Punkt");
			}
			
			if (umWeg < stopRadWP) {
				obsX = pWegPunkt.getX();
				obsY = pWegPunkt.getY();
				wp1 = false;
				wp3 = false;
				counter++;
//				System.out.println("2. Punkt");
			}
			
			if(zielWeg < stopRadWP && !wp3 && !wp2) {
				obsX = pZiel.getX();
				obsY = pZiel.getY();
				hit = false;
				wp1 = true;
				wp2 = true;
				wp3 = false;
				counter = 0;
			}
			//System.out.println("dualWp: " + dualWP);
		}
	}
	
	private int quadTr11(float x, float y) {	//Unterteilt Track 11 in vier Quadranten und erkennt in welchem der Punkt des Parameters gerade ist.
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
	
	private int quadTr12(float x, float y) {	//Unterteilt Track 12 in sechs Quadranten und erkennt in welchem der Punkt des Parameters gerade ist.
		int quad;
		if(x < width*1./3. && y > 500) {
			quad = 1;
		} else if(x < width*2./3. && x > width*1./3. && y > 500) {
			quad = 2;
		} else if(x > width*2./3.&& y > 500) {
			quad = 3;
		} else if(x < width*1./3. && y < 500) {
			quad = 4;
		} else if(x < width*2./3. && x > width*1./3. && y < 500){
			quad = 5;
		} else if(x > width*2./3.&& y < 500) {
			quad = 6;
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
	
	private void oriWinkCP() {
		zielWinkelCP = atan2Vector(x, y, checkPX, checkPY);
		rotZielWinkelCP = zielWinkelCP - ori;
		if (rotZielWinkelCP >= Math.PI) {
			rotZielWinkelCP = (float) (rotZielWinkelCP - 2*Math.PI);
		} else if (rotZielWinkelCP <= -Math.PI) {
			rotZielWinkelCP = (float) (rotZielWinkelCP + 2*Math.PI);
		}
		rotZielAbsWinkelCP = Math.abs(rotZielWinkelCP);
	}

	@Override
	public String getTextureResourceName() {
		return "/s0557060/car.png";
	}

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
			wunschGeschw = bremsGeschw;
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

		return richtung;
	}

	public void doDebugStuff() {

		//Darstellung des Rasters
		GL11.glLineWidth(3f);
		GL11.glBegin(GL11.GL_LINES);
		for(int i = 0; i < width; i += cellSize) {
			for(int j = 0; j < height; j += cellSize) {
				GL11.glColor3d(0, 0, 0);
//				GL11.glVertex2f(i, j + 50);
//				GL11.glVertex2f(i , j);
//				GL11.glVertex2f(i + 50, j);
//				GL11.glVertex2f(i + 50, j + 50);
				GL11.glVertex2d(i, 0);
				GL11.glVertex2d(i, height);
				GL11.glVertex2d(0, j);
				GL11.glVertex2d(width, j);
			}
		}
		GL11.glEnd();
		
//		GL11.glBegin(GL11.GL_QUADS);
//		for(int i = 0; i < width; i += cellSize) {
//			for(int j = 0; j < height; j += cellSize) {
//				if(!arrB[i/cellSize][j/cellSize]) {
//					GL11.glColor3d(0, 0, 1f);
//				} else {
//					GL11.glColor3d(1, 0, 0);
//				}
//				GL11.glVertex2f(i + 1, j + 1);
//				GL11.glVertex2f(i + cellSize - 1, j + 1);
//				GL11.glVertex2f(i + cellSize - 1, j + cellSize - 1);
//				GL11.glVertex2f(i + 1, j + cellSize - 1);
//			}
//		}
//		GL11.glEnd();
		
		//Darstellung der Kanten
		ArrayList<Edge> edges;
		int endPx = (int)(checkPX/cellSize);
		int endPy = (int)(checkPY/cellSize);
		//List<Node> path = printPath(nodes[endPx][endPy]);
		
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(255, 255, 0);
		for(int i = 0 ; i < nodes.length; i++) {
			for(int j = 0; j < nodes.length; j++) {
				edges = nodes[i][j].getEdgeArr();
				if(edges == null) {
					continue;
				}
				for(Edge e : edges) {
					GL11.glVertex2d(nodes[i][j].nodeX, nodes[i][j].nodeY);
					GL11.glVertex2d(e.targetX, e.targetY);
				}
			}
		}
		GL11.glEnd();
		
		List<Node> path = printPath(nodes[endPx][endPy]);	//node[endPx][endPy]	//Test: [47][47]
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3d(0, 0, 1);
		for(Node e : path) {
			String name = e.value;
	    	String[] parts = name.split(" ");
	    	String part1 = parts[0];
	    	String part2 = parts[1];
	    	int part1Int = Integer.parseInt(part1);
	    	int part2Int = Integer.parseInt(part2);
			GL11.glVertex2d(part1Int, part2Int);
		}
		GL11.glEnd();
		
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

		zielCoordY = (float) (by - ay);
		zielCoordlX = (float) (bx - ax);
		
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

class Node{

    public final String value;
    public double g_scores;
    public final double h_scores;
    public double f_scores = 0;
    public ArrayList<Edge> adjacencies;
    public Node parent;
    public float nodeX;
    public float nodeY;

    public Node(String val, double hVal){
            value = val;
            h_scores = hVal;
            String name = value;
        	String[] parts = name.split(" ");
        	String part1 = parts[0];
        	String part2 = parts[1];
        	this.nodeX= Float.parseFloat(part1) * MyAi.cellSize + MyAi.cellSize/2;
        	this.nodeY = Float.parseFloat(part2) * MyAi.cellSize + MyAi.cellSize/2;
            
    }
    
    public ArrayList<Edge> getEdgeArr() {
    	ArrayList<Edge> edges = (ArrayList) adjacencies.clone();
    	return edges;
    }

    public String toString(){
            return value;
    }

}

class Edge{
    public double cost;
    public final Node target;
    public float targetX;
    public float targetY;

    public Edge(Node targetNode, double costVal){
            target = targetNode;
            cost = costVal;
            this.targetX = targetNode.nodeX;
            this.targetY = targetNode.nodeY;
            //System.out.println(" Source: " + target.nodeX + "-" + target.nodeY + " | "  + "Target: " + targetX + "-" + targetY + " ");
    }
}
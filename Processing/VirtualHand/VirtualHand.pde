//Import Libraries
import processing.net.*;
import toxi.geom.*;
import toxi.processing.*;
ToxiclibsSupport gfx;

//Simple class for 3D coordinates
public class coord3D {
	public float x;
	public float y;
	public float z;
}

//Class for 360 view
public class sphericView {
	public float rotX;
	public float rotY;
	public float zoom;
}

//Finger class to indicate coordinate and contact
public class finger {
	public coord3D coord = new coord3D();
	public boolean contact;
}

//Used for syncing the messages 
boolean readIsOk;

//Used for socket communication
int socket_port=7891;
Server myServer;
Client thisClient;
char charIn;

// Packet received with IMU data
char[] quaternionPacket = new char[20];

//Quaternion, Acceleration and Yaw, Pitch and Row
float[] q_MPU1 = new float[4];
float[] gravity_MPU1 = new float[3];      
float[] ypr_MPU1 = new float[3];          
float[] q_MPU2 = new float[4];
float[] gravity_MPU2 = new float[3];      
float[] ypr_MPU2 = new float[3];          

//Toxilibs quaternion
Quaternion quat = new Quaternion(1, 0, 0, 0);

//Coordinates of obstable
coord3D obstacle_sphere = new coord3D();

//Used for 360 view
sphericView sView = new sphericView();

//Arm and hand angle
coord3D armAngle = new coord3D();
coord3D handAngle = new coord3D();

//Hand XYZ coordinates
coord3D handXYZ= new coord3D();

//Arm Dimentions
coord3D arm = new coord3D();

void setup() {
	//Creates screen
	//size(640, 360, P3D);
	fullScreen(P3D);

  	//Sets up initial angles
	sView.rotX=0;
	sView.rotY=0;
	sView.zoom=1;
	armAngle.x=0;
	armAngle.y=0;
	handAngle.x=0;
	handAngle.z=0;

  	// Starts a server on port socket_port
	myServer = new Server(this, socket_port);

  	//Set receive flag to false
	readIsOk=false;

	//Arm Dimentions
	arm.x=35;
	arm.y=200;
	arm.z=20;
}


void draw() {

	//Checks server
	thisClient = myServer.available();


  	//If server is available read the message
	if (thisClient !=null) {
		if (thisClient.available() > 0) {
			charIn = thisClient.readChar();
			if (charIn=='$') {
				readIsOk=true;
				quaternionPacket[0]='$';
				for (int i=1; i<quaternionPacket.length; i++) {
					if (thisClient.available() > 0) {
						charIn = thisClient.readChar();
						quaternionPacket[i]=charIn;
					}
				}
			}
			interpretMessage();
		}
	}

	//Black background
	background(0);

	//Press keys for moving hand and view (w,a,s,d,+ and - are your best friends)
	if (keyPressed) {
		if (key == 's') {
			sView.rotX=sView.rotX-0.02;
		} else if (key == 'w') {
			sView.rotX=sView.rotX+0.02;
		}
		if (key == 'a') {
			sView.rotY=sView.rotY+0.02;
		} else if (key == 'd') {
			sView.rotY=sView.rotY-0.02;
		} 
		if (key == '-') {
			sView.zoom=sView.zoom+0.02;
		} else if (key == '=') {
			sView.zoom=sView.zoom-0.02;
		} 
		if (key == 'z' && armAngle.x<PI/2) {
			armAngle.x=armAngle.x+0.02;
		} else if (key == 'x' && armAngle.x>0) {
			armAngle.x=armAngle.x-0.02;
		} 
		if (key == 'v' && armAngle.y<PI) {
			armAngle.y=armAngle.y+0.02;
		} else if (key == 'c' && armAngle.y>0) {
			armAngle.y=armAngle.y-0.02;
		} 
		if (key == 'b' && handAngle.x<PI/2) {
			handAngle.x=handAngle.x+0.02;
		} else if (key == 'n' && handAngle.x>-PI/4) {
			handAngle.x=handAngle.x-0.02;
		} 
		if (key == 'm' && handAngle.z<PI/4) {
			handAngle.z=handAngle.z+0.02;
		} else if (key == ',' && handAngle.z>-PI/4) {
			handAngle.z=handAngle.z-0.02;
		} 

		float cameraZ = height/2.0 / tan(sView.zoom / 2.0);
		float aspect = float(width)/float(height);
		perspective(sView.zoom, aspect, cameraZ/10.0, cameraZ*10.0);
	} 
	translate(width/2, height*0.7, -100);
	rotateX(sView.rotX);
	rotateY(sView.rotY);
	stroke(255);
	noFill();

	//Radius of sphere that represents the wrist
	int wristRadius=10;

	//Create finger struct array
	finger[] fing = new finger[5];
	for (int i=0; i<5; i++) {
		fing[i]=new finger();
	}

	//The arm angle is given by the IMU data
	armAngle.x=ypr_MPU2[2]+PI/2;
	armAngle.y=ypr_MPU2[1];

	//The hand angle is given by the IMU data (needs improvement)
	handAngle.x=(ypr_MPU1[2]+PI/2)-armAngle.x;

	//Pushes the current transformation matrix onto the matrix stack
	pushMatrix();

	//Beautify
	directionalLight(250, 253, 251, -24, 100, 15);
	fill(color(84, 119, 168));

	//Gets matrix to use this point as reference
	Matrix refMatrix = getReferenceMatrix();

	//Creates Arm
	rotateX(armAngle.x);
	rotateY(armAngle.y);
	translate(0, -arm.y/2, 0);
	box(arm.x, arm.y, arm.z);

	//Creates Hand and wrist
	translate(0, -arm.y/2-wristRadius, 0);
  	rotateX(handAngle.x);
  	rotateZ(handAngle.z);
  	sphere(wristRadius);
  	translate(0, -3*wristRadius, 0);
  	box(arm.x, arm.x, arm.z);

  	//Creates Thumb
  	float esferaDedao=6.6;
  	translate(-29, 7, 0);
  	translate(-11, 0, 0);
  	fing[0].coord=get3DPoint(refMatrix);
  	sphere(esferaDedao);
  	translate(11, 0, 0);
  	box(22, 10, 10);

  	//Creates Pinky
  	float aux=-14;
  	float fingerSphereRadius=3.5;
  	translate(42, -42, 0);
  	translate(0, aux, 0);
  	fing[1].coord=get3DPoint(refMatrix);
  	sphere(fingerSphereRadius);
  	translate(0, -aux, 0);
  	box(6, 30, 6);

  	//Creates Ring
  	translate(-8, 0, 0);
  	translate(0, aux, 0);
  	fing[2].coord=get3DPoint(refMatrix);
  	sphere(fingerSphereRadius);
  	translate(0, -aux, 0);
  	box(6, 30, 6);

  	//Creates Middle
  	translate(-8, 0, 0);
  	translate(0, aux, 0);
  	fing[3].coord=get3DPoint(refMatrix);
  	sphere(fingerSphereRadius);
  	translate(0, -aux, 0);
  	box(6, 30, 6);

  	//Creates Index
  	translate(-8, 0, 0);
  	translate(0, aux, 0);
  	fing[4].coord=get3DPoint(refMatrix);
  	sphere(fingerSphereRadius);
  	translate(0, -aux, 0);
  	box(6, 30, 6);

  	//Pops the current transformation matrix off the matrix stack
  	popMatrix();
  	
  	//Just for beauty
  	lights();

  	rectMode(CENTER);
  	rotateX(PI/2);

  	//Draws ground
  	rect(0, 0, 400, 400);
  	fill(color(2, 49, 223)); 
  	translate(0, -219, 132);

  	//Draws obstacle
  	fill(color(0, 255, 24)); 
  	translate(0, -28, -138);
  	int obstacleSphereRadius=55;
  	sphere(obstacleSphereRadius);
  	obstacle_sphere=get3DPoint(refMatrix);

  	//Detect colision and build packet
  	int bitout=0;
  	for (int i=0; i<5; i++) {
  		if ((two_3D_points_distance(obstacle_sphere, fing[i].coord))<=(obstacleSphereRadius+fingerSphereRadius)) {
  			bitout=bitout|(1<<i);
  		} 
  	}

  	//If everything is ok we can send the message
  	if(readIsOk){
  		myServer.write(bitout);
  		readIsOk=false;
  	}

}

//Function for getting the Matrix that is going to be used as reference
Matrix getReferenceMatrix() {
	PMatrix3D m = (PMatrix3D)getMatrix();
	double[][] array = {{m.m00, m.m01, m.m02, m.m03}, {m.m10, m.m11, m.m12, m.m13}, {m.m20, m.m21, m.m22, m.m23}, {m.m30, m.m31, m.m32, m.m33}};
	Matrix M= new Matrix(array);
	if (M.det()!=0) {
		M=M.inverse();
	}
	return M;
}

//Get XYZ coordinates of current point when compared to the reference matrix
coord3D get3DPoint(Matrix refMatrix) {
	PMatrix3D m = (PMatrix3D)getMatrix();
	double[][] array = {{m.m00, m.m01, m.m02, m.m03}, {m.m10, m.m11, m.m12, m.m13}, {m.m20, m.m21, m.m22, m.m23}, {m.m30, m.m31, m.m32, m.m33}};
	Matrix M = new Matrix(array);
	M=refMatrix.times(M);
	coord3D coord= new coord3D();
	coord.x = (float)M.get(0, 3);
	coord.y = (float)M.get(1, 3);
	coord.z = (float)M.get(2, 3);
	return coord;
}

//Get distance between two 3d points
float two_3D_points_distance(coord3D coord1, coord3D coord2) {
	return sqrt(((coord1.x-coord2.x)*(coord1.x-coord2.x))+((coord1.y-coord2.y)*(coord1.y-coord2.y))+((coord1.z-coord2.z)*(coord1.z-coord2.z)));
}

//Interpret the IMU incoming message
void interpretMessage() {

	//Checks if the beginning and the end of the message are ok
	if (quaternionPacket[0]=='$' && quaternionPacket[1]==0x02 && quaternionPacket[18]=='\r' && quaternionPacket[19]=='\n') {
		println("Message is ok\n");

		//Reads quaternion data
		q_MPU1[0] = ((quaternionPacket[2] << 8) | quaternionPacket[3]) / 16384.0f;
		q_MPU1[1] = ((quaternionPacket[4] << 8) | quaternionPacket[5]) / 16384.0f;
		q_MPU1[2] = ((quaternionPacket[6] << 8) | quaternionPacket[7]) / 16384.0f;
		q_MPU1[3] = ((quaternionPacket[8] << 8) | quaternionPacket[9]) / 16384.0f;
		q_MPU2[0] = ((quaternionPacket[10] << 8) | quaternionPacket[11]) / 16384.0f;
		q_MPU2[1] = ((quaternionPacket[12] << 8) | quaternionPacket[13]) / 16384.0f;
		q_MPU2[2] = ((quaternionPacket[14] << 8) | quaternionPacket[15]) / 16384.0f;
		q_MPU2[3] = ((quaternionPacket[16] << 8) | quaternionPacket[17]) / 16384.0f;
		for (int i = 0; i < 4; i++) if (q_MPU1[i] >= 2) q_MPU1[i] = -4 + q_MPU1[i];
		for (int i = 0; i < 4; i++) if (q_MPU2[i] >= 2) q_MPU2[i] = -4 + q_MPU2[i];


    	// set our toxilibs quaternion to new data
		quat.set(q_MPU1[0], q_MPU1[1], q_MPU1[2], q_MPU1[3]);
		quat.set(q_MPU2[0], q_MPU2[1], q_MPU2[2], q_MPU2[3]);



    	// calculate gravity vector
		gravity_MPU1[0] = 2 * (q_MPU1[1]*q_MPU1[3] - q_MPU1[0]*q_MPU1[2]);
		gravity_MPU1[1] = 2 * (q_MPU1[0]*q_MPU1[1] + q_MPU1[2]*q_MPU1[3]);
		gravity_MPU1[2] = q_MPU1[0]*q_MPU1[0] - q_MPU1[1]*q_MPU1[1] - q_MPU1[2]*q_MPU1[2] + q_MPU1[3]*q_MPU1[3];
		gravity_MPU2[0] = 2 * (q_MPU2[1]*q_MPU2[3] - q_MPU2[0]*q_MPU2[2]);
		gravity_MPU2[1] = 2 * (q_MPU2[0]*q_MPU2[1] + q_MPU2[2]*q_MPU2[3]);
		gravity_MPU2[2] = q_MPU2[0]*q_MPU2[0] - q_MPU2[1]*q_MPU2[1] - q_MPU2[2]*q_MPU2[2] + q_MPU2[3]*q_MPU2[3];


    	// calculate yaw/pitch/roll angles
		ypr_MPU1[0] = atan2(2*q_MPU1[1]*q_MPU1[2] - 2*q_MPU1[0]*q_MPU1[3], 2*q_MPU1[0]*q_MPU1[0] + 2*q_MPU1[1]*q_MPU1[1] - 1);
		ypr_MPU1[1] = atan(gravity_MPU1[0] / sqrt(gravity_MPU1[1]*gravity_MPU1[1] + gravity_MPU1[2]*gravity_MPU1[2]));
		ypr_MPU1[2] = atan(gravity_MPU1[1] / sqrt(gravity_MPU1[0]*gravity_MPU1[0] + gravity_MPU1[2]*gravity_MPU1[2]));
		ypr_MPU2[0] = atan2(2*q_MPU2[1]*q_MPU2[2] - 2*q_MPU2[0]*q_MPU2[3], 2*q_MPU2[0]*q_MPU2[0] + 2*q_MPU2[1]*q_MPU2[1] - 1);
		ypr_MPU2[1] = atan(gravity_MPU2[0] / sqrt(gravity_MPU2[1]*gravity_MPU2[1] + gravity_MPU2[2]*gravity_MPU2[2]));
		ypr_MPU2[2] = atan(gravity_MPU2[1] / sqrt(gravity_MPU2[0]*gravity_MPU2[0] + gravity_MPU2[2]*gravity_MPU2[2]));

	} else {
		println("The message was lost!");
	}
}

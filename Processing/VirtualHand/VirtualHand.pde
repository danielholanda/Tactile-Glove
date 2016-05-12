import processing.net.*;
import toxi.geom.*;
import toxi.processing.*;
ToxiclibsSupport gfx;

boolean readIsOk;
String whatClientSaid;
int socket_port=7891;
Server myServer;   
char[] teapotPacket = new char[20];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int aligned = 0;
int interval = 0;
int lastMillis=0;                      //Used for detecting frequency.
int imuFreqCounter=0;
int imuFreq=0;
float[] q_MPU1 = new float[4];
float[] gravity_MPU1 = new float[3];      //Acceleration
float[] ypr_MPU1 = new float[3];          //Yaw, Pitch and Row

float[] q_MPU2 = new float[4];
float[] gravity_MPU2 = new float[3];      //Acceleration
float[] ypr_MPU2 = new float[3];          //Yaw, Pitch and Row

float[] g = new float[3];            //Acceleration without gravity
float[] compensation = new float[4]; //Used for zeroing the quaternion after calibration.
float shipDisplacement=0;            //Displacement in relation to water level in m
float aaWorldz= 0;
float speedZ=0;
float aaWorldzOffset=0;              //Offset from zero
Quaternion quat = new Quaternion(1, 0, 0, 0);
Client thisClient;
char charIn;

public class coord3D {
  public float x;
  public float y;
  public float z;
}


public class sphericView {
  public float rotX;
  public float rotY;
  public float zoom;
}

public class finger {
  public coord3D coord = new coord3D();
  public boolean contact;
}



coord3D obstacle_sphere = new coord3D();
boolean contact_obstacle_sphere;


sphericView sView = new sphericView();//Para visão 360
coord3D armAngle = new coord3D();//Angulo do braço
coord3D handAngle = new coord3D();//Angulo do braço

coord3D handXYZ= new coord3D();

void setup() {
  size(640, 360, P3D);
  //fullScreen(P3D);
  sView.rotX=0;
  sView.rotY=0;
  sView.zoom=1;
  armAngle.x=0;
  armAngle.y=0;
  handAngle.x=0;
  handAngle.z=0;
  myServer = new Server(this, socket_port); // Starts a server on port socket_port
  readIsOk=false;
}


void draw() {

  thisClient = myServer.available();
  while (thisClient ==null) {
    thisClient = myServer.available();
  };
  if (thisClient !=null) {

    //whatClientSaid = thisClient.readString();

    if (thisClient.available() > 0) {
      charIn = thisClient.readChar();
      if (charIn=='$') {
        readIsOk=true;
        teapotPacket[0]='$';
        println("Começando a mensagem");
        for (int i=1; i<teapotPacket.length; i++) {
          if (thisClient.available() > 0) {
            charIn = thisClient.readChar();
            teapotPacket[i]=charIn;
          }
        }
      }

      tratarMensagem();
    }
  }

  



  background(0);
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

  coord3D arm = new coord3D();
  arm.x=35;//Punho
  arm.y=200;//Comprimento do braço
  arm.z=20;//largura do braço

  int sphereDiameter=10;

  //Create finger struct array
  finger[] fing = new finger[5];
  for (int i=0; i<5; i++) {
    fing[i]=new finger();
  }

  armAngle.x=ypr_MPU2[2]+PI/2;
  armAngle.y=ypr_MPU2[1];

  handAngle.x=(ypr_MPU1[2]+PI/2)-armAngle.x;
  //handAngle.y=(ypr_MPU1[1])-armAngle.y;
  handAngle.z=(ypr_MPU1[0])-armAngle.x;


  pushMatrix();
  directionalLight(250, 253, 251, -24, 100, 15);
  fill(color(84, 119, 168));
  Matrix refMatrix = getReferenceMatrix();
  rotateX(armAngle.x);
  rotateY(armAngle.y);
  translate(0, -arm.y/2, 0);
  box(arm.x, arm.y, arm.z);//Braço
  translate(0, -arm.y/2-sphereDiameter, 0);
  rotateX(handAngle.x);
  rotateZ(handAngle.z);
  sphere(sphereDiameter);
  translate(0, -3*sphereDiameter, 0);
  fing[0].coord=get3DPoint(refMatrix);
  box(arm.x, arm.x, arm.z);//Mao

  //Dedo 1 (Dedão)
  float esferaDedao=6.6;
  translate(-29, 7, 0);
  translate(-11, 0, 0);
  fing[0].coord=get3DPoint(refMatrix);
  sphere(esferaDedao);//Tamanho da esfera do dedão
  translate(11, 0, 0);
  box(22, 10, 10);

  //Dedo 2
  float aux=-14;
  float esferaDedinhos=3.5;
  translate(42, -42, 0);
  translate(0, aux, 0);
  fing[1].coord=get3DPoint(refMatrix);
  sphere(esferaDedinhos);
  translate(0, -aux, 0);
  box(6, 30, 6);//Dedinhos

  //Dedo 3
  translate(-8, 0, 0);
  translate(0, aux, 0);
  fing[2].coord=get3DPoint(refMatrix);
  sphere(esferaDedinhos);
  translate(0, -aux, 0);
  box(6, 30, 6);//Dedinhos

  //Dedo 4
  translate(-8, 0, 0);
  translate(0, aux, 0);
  fing[3].coord=get3DPoint(refMatrix);
  sphere(esferaDedinhos);
  translate(0, -aux, 0);
  box(6, 30, 6);//Dedinhos

  //Dedo 5
  translate(-8, 0, 0);
  translate(0, aux, 0);
  fing[4].coord=get3DPoint(refMatrix);
  sphere(esferaDedinhos);
  translate(0, -aux, 0);
  box(6, 30, 6);//Dedinhos

  popMatrix();
  lights();

  rectMode(CENTER);
  rotateX(PI/2);

  rect(0, 0, 400, 400);

  fill(color(2, 49, 223)); 
  translate(0, -219, 132);


  fill(color(0, 255, 24)); 
  translate(0, -28, -138);
  int sphere_radius=55;
  sphere(sphere_radius);
  obstacle_sphere=get3DPoint(refMatrix);


  boolean[] dedos = new boolean[5];
  //boolean tipo=false;


  for (int i=0; i<5; i++) {
    if ((two_3D_points_distance(obstacle_sphere, fing[i].coord))<=(sphere_radius+esferaDedinhos)) {
      dedos[i]=true;
      //println("\n Dedo " + i + " em contato \n ");
    } else
      dedos[i]=false;
  }

  int bitout=0;
  for (int i=0; i<5; i++) {
    if (dedos[i])
      bitout=bitout|(1<<i);
  }

  if(readIsOk){
    myServer.write(bitout);
    readIsOk=false;
  }
  //port.write(char(bitout));

  //print((bitout));print((bitout));print((bitout));print((bitout));print((bitout));print((bitout));
  //println(dedos[0]+" | "+dedos[1]+" | "+dedos[2]+" | "+dedos[3]+" | "+dedos[4]+" - TIPO: "+tipo+ " bitout:"+bitout+" AngleY: "+armAngle.y);
}


Matrix getReferenceMatrix() {
  PMatrix3D m = (PMatrix3D)getMatrix();
  double[][] array = {{m.m00, m.m01, m.m02, m.m03}, {m.m10, m.m11, m.m12, m.m13}, {m.m20, m.m21, m.m22, m.m23}, {m.m30, m.m31, m.m32, m.m33}};
  Matrix M= new Matrix(array);
  if (M.det()!=0) {
    M=M.inverse();
  }
  return M;
}


float two_3D_points_distance(coord3D coord1, coord3D coord2) {
  return sqrt(((coord1.x-coord2.x)*(coord1.x-coord2.x))+((coord1.y-coord2.y)*(coord1.y-coord2.y))+((coord1.z-coord2.z)*(coord1.z-coord2.z)));
}

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

void tratarMensagem() {




  if (teapotPacket[0]=='$' && teapotPacket[1]==0x02 && teapotPacket[18]=='\r' && teapotPacket[19]=='\n') {
    println("Mensagem ok\n");
    // MPU5060 1
    q_MPU1[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
    q_MPU1[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
    q_MPU1[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
    q_MPU1[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;

    // MPU5060 2
    q_MPU2[0] = ((teapotPacket[10] << 8) | teapotPacket[11]) / 16384.0f;
    q_MPU2[1] = ((teapotPacket[12] << 8) | teapotPacket[13]) / 16384.0f;
    q_MPU2[2] = ((teapotPacket[14] << 8) | teapotPacket[15]) / 16384.0f;
    q_MPU2[3] = ((teapotPacket[16] << 8) | teapotPacket[17]) / 16384.0f;

    for (int i = 0; i < 4; i++) if (q_MPU1[i] >= 2) q_MPU1[i] = -4 + q_MPU1[i];

    for (int i = 0; i < 4; i++) if (q_MPU2[i] >= 2) q_MPU2[i] = -4 + q_MPU2[i];

    //Concertar compensação 
    //quat.set(q[0]-compensation[0]+1, q[1]-compensation[1], q[2]-compensation[2], q[3]-compensation[3]);
    //quat[0] é no max 1 -> fazer alguma matematica inteligente para isso



    // set our toxilibs quaternion to new data
    //MPU - 1
    quat.set(q_MPU1[0], q_MPU1[1], q_MPU1[2], q_MPU1[3]);
    //MPU - 2
    quat.set(q_MPU2[0], q_MPU2[1], q_MPU2[2], q_MPU2[3]);


    // below calculations unnecessary for orientation only using toxilibs

    // calculate gravity vector
    //MPU - 1
    gravity_MPU1[0] = 2 * (q_MPU1[1]*q_MPU1[3] - q_MPU1[0]*q_MPU1[2]);
    gravity_MPU1[1] = 2 * (q_MPU1[0]*q_MPU1[1] + q_MPU1[2]*q_MPU1[3]);
    gravity_MPU1[2] = q_MPU1[0]*q_MPU1[0] - q_MPU1[1]*q_MPU1[1] - q_MPU1[2]*q_MPU1[2] + q_MPU1[3]*q_MPU1[3];

    // calculate gravity vector
    //MPU - 2
    gravity_MPU2[0] = 2 * (q_MPU2[1]*q_MPU2[3] - q_MPU2[0]*q_MPU2[2]);
    gravity_MPU2[1] = 2 * (q_MPU2[0]*q_MPU2[1] + q_MPU2[2]*q_MPU2[3]);
    gravity_MPU2[2] = q_MPU2[0]*q_MPU2[0] - q_MPU2[1]*q_MPU2[1] - q_MPU2[2]*q_MPU2[2] + q_MPU2[3]*q_MPU2[3];


    // calculate yaw/pitch/roll angles
    //MPU - 1
    ypr_MPU1[0] = atan2(2*q_MPU1[1]*q_MPU1[2] - 2*q_MPU1[0]*q_MPU1[3], 2*q_MPU1[0]*q_MPU1[0] + 2*q_MPU1[1]*q_MPU1[1] - 1);
    ypr_MPU1[1] = atan(gravity_MPU1[0] / sqrt(gravity_MPU1[1]*gravity_MPU1[1] + gravity_MPU1[2]*gravity_MPU1[2]));
    ypr_MPU1[2] = atan(gravity_MPU1[1] / sqrt(gravity_MPU1[0]*gravity_MPU1[0] + gravity_MPU1[2]*gravity_MPU1[2]));

    //MPU - 2
    ypr_MPU2[0] = atan2(2*q_MPU2[1]*q_MPU2[2] - 2*q_MPU2[0]*q_MPU2[3], 2*q_MPU2[0]*q_MPU2[0] + 2*q_MPU2[1]*q_MPU2[1] - 1);
    ypr_MPU2[1] = atan(gravity_MPU2[0] / sqrt(gravity_MPU2[1]*gravity_MPU2[1] + gravity_MPU2[2]*gravity_MPU2[2]));
    ypr_MPU2[2] = atan(gravity_MPU2[1] / sqrt(gravity_MPU2[0]*gravity_MPU2[0] + gravity_MPU2[2]*gravity_MPU2[2]));

    //Frequency increment
    imuFreqCounter++;
  } else {
    println("XXXXXXXXXXXXXX   -   Mensagem perdida - XXXXXXXXXXXXXXXXXXXXXXX");
  }
}

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;


ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;

float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];

void setup() {
    // 300px square viewport using OpenGL rendering
    size(1000, 1000, OPENGL);
    gfx = new ToxiclibsSupport(this);

    // setup lights and antialiasing
    lights();
    smooth();
  
    // display serial port list for debugging/clarity
    println(Serial.list());

    // get the first available port (use EITHER this OR the specific port code below)
    String portName = Serial.list()[0];
    
    // get a specific serial port (use EITHER this OR the first-available code above)
    //String portName = "COM4";
    
    // open the serial port
    port = new Serial(this, portName, 115200);
    
    // send single character to trigger DMP init/start
    // (expected by MPU6050_DMP6 example Arduino sketch)
    port.write('r');
}

void draw() {
    if (millis() - interval > 1000) {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
        port.write('r');
        interval = millis();
    }
    
    // black background
    background(0);
    
    // translate everything to the middle of the viewport
    pushMatrix();
    translate(width / 2, height / 2);

    // 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
    // ...and other weirdness I haven't figured out yet
    //rotateY(-ypr[0]);
    //rotateZ(-ypr[1]);
    //rotateX(-ypr[2]);

    // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
    // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
    // different coordinate system orientation assumptions between Processing
    // and InvenSense DMP)
    float[] axis = quat.toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);
    drawPlane(); 
    ailerons_right();
    ailerons_left();
    rudder();
    elevator();
    popMatrix();
}

void serialEvent(Serial port) {
    interval = millis();
    while (port.available() > 0) {
        int ch = port.read();

        if (synced == 0 && ch != '$') return;   // initial synchronization - also used to resync/realign if needed
        synced = 1;
        print ((char)ch);

        if ((serialCount == 1 && ch != 2)
            || (serialCount == 12 && ch != '\r')
            || (serialCount == 13 && ch != '\n'))  {
            serialCount = 0;
            synced = 0;
            return;
        }

        if (serialCount > 0 || ch == '$') {
            teapotPacket[serialCount++] = (char)ch;
            if (serialCount == 14) {
                serialCount = 0; // restart packet byte position
                
                // get quaternion from data packet
                q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
                q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
                q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
                q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
                for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
                
                // set our toxilibs quaternion to new data
                quat.set(q[0], q[1], q[2], q[3]);

                /*
                // below calculations unnecessary for orientation only using toxilibs
                
                // calculate gravity vector
                gravity[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
                gravity[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
                gravity[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
                // calculate Euler angles
                euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
                euler[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);
                euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
    
                // calculate yaw/pitch/roll angles
                ypr[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
                ypr[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
                ypr[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
    
                // output various components for debugging
                //println("q:\t" + round(q[0]*100.0f)/100.0f + "\t" + round(q[1]*100.0f)/100.0f + "\t" + round(q[2]*100.0f)/100.0f + "\t" + round(q[3]*100.0f)/100.0f);
                //println("euler:\t" + euler[0]*180.0f/PI + "\t" + euler[1]*180.0f/PI + "\t" + euler[2]*180.0f/PI);
                //println("ypr:\t" + ypr[0]*180.0f/PI + "\t" + ypr[1]*180.0f/PI + "\t" + ypr[2]*180.0f/PI);
                */
            }
        }
    }
}

void drawPlane() {
  // Body of the plane
  fill(0,0,255);
  box(185, 10, 10);  // Main fuselage

  // Left wing
  pushMatrix();
  translate(-30, -5, 0);
  fill(0,0,255);
  box(30,3,180);  
  popMatrix();
  
  //tail wing
  pushMatrix();
  translate(80,0,0);
  fill(0,0,255);
  box(25,3,90);  
  popMatrix();
  
  //tail
  fill(0,0,255);
  beginShape();  
  vertex(65, -5, 1.5);
  vertex(95, -5, 1.5);
  vertex(95, -40, 1.5);
  endShape(CLOSE); 
  
  beginShape();
  vertex(65, -5, -1.5);
  vertex(95, -5, -1.5);
  vertex(95, -40, -1.5);
  endShape(CLOSE); 
  
  beginShape();
  vertex(65, -5, 1.5);
  vertex(65, -5, -1.5);
  vertex(95, -40, -1.5);
  vertex(95, -40, 1.5);
  endShape(CLOSE);
 
}
void ailerons_left() {
  // ailerons left
  pushMatrix();
  translate(-10, -5, 50); // Translate to the position for the ailerons

  fill(255, 0, 0); // Set fill color to red
  
  // Draw the box using vertices
  beginShape();
  // Front face
  vertex(-5, -1.5, -40); // Vertex 1
  vertex(-5, -1.5, 40);  // Vertex 2
  vertex(5, -1.5, 40);   // Vertex 3
  vertex(5, -1.5, -40);  // Vertex 4
  endShape(CLOSE);
  
  beginShape();
  // Back face
  vertex(-5, 1.5, -40);  // Vertex 1
  vertex(-5, 1.5, 40);   // Vertex 2
  vertex(5, 1.5, 40);    // Vertex 3
  vertex(5, 1.5, -40);   // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Left face
  vertex(-5, -1.5, -40); // Vertex 1
  vertex(-5, 1.5, -40);  // Vertex 2
  vertex(-5, 1.5, 40);   // Vertex 3
  vertex(-5, -1.5, 40);  // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Right face
  vertex(5, -1.5, -40);  // Vertex 1
  vertex(5, 1.5, -40);   // Vertex 2
  vertex(5, 1.5, 40);    // Vertex 3
  vertex(5, -1.5, 40);   // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Top face
  vertex(-5, -1.5, 40);  // Vertex 1
  vertex(-5, 1.5, 40);   // Vertex 2
  vertex(5, 1.5, 40);    // Vertex 3
  vertex(5, -1.5, 40);   // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Bottom face
  vertex(-5, -1.5, -40); // Vertex 1
  vertex(-5, 1.5, -40);  // Vertex 2
  vertex(5, 1.5, -40);   // Vertex 3
  vertex(5, -1.5, -40);  // Vertex 4
  endShape(CLOSE);

  popMatrix(); // Restore the saved transformation matrix
}

void ailerons_right() {
  // ailerons right
  pushMatrix();
  translate(-10, -5, -50); // Translate to the position for the ailerons

  fill(255, 0, 0); // Set fill color to red

  // Draw the box using vertices
  beginShape();
  // Front face
  vertex(-5, -1.5, -40); // Vertex 1
  vertex(-5, -1.5, 40);  // Vertex 2
  vertex(5, -1.5, 40);   // Vertex 3
  vertex(5, -1.5, -40);  // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Back face
  vertex(-5, 1.5, -40);  // Vertex 1
  vertex(-5, 1.5, 40);   // Vertex 2
  vertex(5, 1.5, 40);    // Vertex 3
  vertex(5, 1.5, -40);   // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Left face
  vertex(-5, -1.5, -40); // Vertex 1
  vertex(-5, 1.5, -40);  // Vertex 2
  vertex(-5, 1.5, 40);   // Vertex 3
  vertex(-5, -1.5, 40);  // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Right face
  vertex(5, -1.5, -40);  // Vertex 1
  vertex(5, 1.5, -40);   // Vertex 2
  vertex(5, 1.5, 40);    // Vertex 3
  vertex(5, -1.5, 40);   // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Top face
  vertex(-5, -1.5, 40);  // Vertex 1
  vertex(-5, 1.5, 40);   // Vertex 2
  vertex(5, 1.5, 40);    // Vertex 3
  vertex(5, -1.5, 40);   // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Bottom face
  vertex(-5, -1.5, -40); // Vertex 1
  vertex(-5, 1.5, -40);  // Vertex 2
  vertex(5, 1.5, -40);   // Vertex 3
  vertex(5, -1.5, -40);  // Vertex 4
  endShape(CLOSE);

  popMatrix(); // Restore the saved transformation matrix
}

void elevator() {
  // elevator
  pushMatrix(); // Translate to the position for the elevator

  fill(255, 0, 0); // Set fill color to red

  // Draw the box using vertices
  beginShape();
  // Front face
  vertex(92.5, -1.5, -45); // Vertex 1
  vertex(92.5, -1.5, 45);  // Vertex 2
  vertex(102.5, -1.5, 45); // Vertex 3
  vertex(102.5, -1.5, -45); // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Back face
  vertex(92.5, 1.5, -45);  // Vertex 1
  vertex(92.5, 1.5, 45);   // Vertex 2
  vertex(102.5, 1.5, 45);  // Vertex 3
  vertex(102.5, 1.5, -45); // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Left face
  vertex(92.5, -1.5, -45); // Vertex 1
  vertex(92.5, 1.5, -45);  // Vertex 2
  vertex(92.5, 1.5, 45);   // Vertex 3
  vertex(92.5, -1.5, 45);  // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Right face
  vertex(102.5, -1.5, -45);  // Vertex 1
  vertex(102.5, 1.5, -45);   // Vertex 2
  vertex(102.5, 1.5, 45);    // Vertex 3
  vertex(102.5, -1.5, 45);   // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Top face
  vertex(92.5, -1.5, 45);  // Vertex 1
  vertex(92.5, 1.5, 45);   // Vertex 2
  vertex(102.5, 1.5, 45);  // Vertex 3
  vertex(102.5, -1.5, 45); // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Bottom face
  vertex(92.5, -1.5, -45); // Vertex 1
  vertex(92.5, 1.5, -45);  // Vertex 2
  vertex(102.5, 1.5, -45); // Vertex 3
  vertex(102.5, -1.5, -45); // Vertex 4
  endShape(CLOSE);

  popMatrix(); // Restore the saved transformation matrix
}


void rudder() {
  // rudder
  pushMatrix();
  fill(255, 0, 0); // Set fill color to red

  // Draw the box using vertices
  beginShape();
  // Front face
  vertex(94, -10, -1.5); // Vertex 1
  vertex(94, -10, 1.5);  // Vertex 2
  vertex(104, -10, 1.5); // Vertex 3
  vertex(104, -10, -1.5);// Vertex 4
  endShape(CLOSE);

  beginShape();
  // Back face
  vertex(94, -40, -1.5);  // Vertex 1
  vertex(94, -40, 1.5);   // Vertex 2
  vertex(104, -40, 1.5);  // Vertex 3
  vertex(104, -40, -1.5); // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Left face
  vertex(94, -10, -1.5); // Vertex 1
  vertex(94, -40, -1.5);  // Vertex 2
  vertex(94, -40, 1.5);   // Vertex 3
  vertex(94, -10, 1.5);  // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Right face
  vertex(104, -10, -1.5); // Vertex 1
  vertex(104, -40, -1.5);  // Vertex 2
  vertex(104, -40, 1.5);   // Vertex 3
  vertex(104, -10, 1.5);  // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Top face
  vertex(94, -10, 1.5);  // Vertex 1
  vertex(94, -40, 1.5);   // Vertex 2
  vertex(104, -40, 1.5);  // Vertex 3
  vertex(104, -10, 1.5); // Vertex 4
  endShape(CLOSE);

  beginShape();
  // Bottom face
  vertex(94, -10, -1.5); // Vertex 1
  vertex(94, -40, -1.5);  // Vertex 2
  vertex(104, -40, -1.5); // Vertex 3
  vertex(104, -10, -1.5);// Vertex 4
endShape(CLOSE);


  popMatrix(); // Restore the saved transformation matrix
}

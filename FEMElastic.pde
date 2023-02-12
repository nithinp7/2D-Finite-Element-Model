import java.util.ArrayList;

int SCR_WIDTH = 2560;
int SCR_HEIGHT = 1280;

float GRAVITY = 00.0f;

// Young's Modulus
// Note: 1Gpa = 1e9 pa
float YM = 1.0E8f;//2.5E6f;
// Poisson Ratio
//float PR = 0.3;//4999f;
float PR = 0.39;

// One minus poisson ratio
float OMPR = 1.0f - PR;
// One minus 2 * Poisson ratio
float OM2PR = 1.0f - 2.0f * PR;

// Useful constant, when computing stress from strain
float C = YM / ((1.0f + PR) * OM2PR);

int GRID_WIDTH = 80;
int GRID_HEIGHT = 80;
float SPACING = 10.0f;

//Node[][] nodes = new Node[GRID_WIDTH][GRID_HEIGHT];
ArrayList<Node> nodes = new ArrayList<Node>();
ArrayList<Element> elements = new ArrayList<Element>();

//Element[][][] elements = new Element[GRID_WIDTH-1][GRID_HEIGHT-1][2];

void settings() {
  size(SCR_WIDTH, SCR_HEIGHT); 
  
  nodes.ensureCapacity(GRID_WIDTH * GRID_HEIGHT);
  for (int i = 0; i < GRID_WIDTH; ++i) {
    for (int j = 0; j < GRID_HEIGHT; ++j) {
      nodes.add(new Node(SPACING * float(i) + 800.0f, SPACING * float(j) + 100.0f));
    }
  }
  
  elements.ensureCapacity((GRID_WIDTH - 1) * (GRID_HEIGHT - 1) * 2); 
  for (int i = 0; i < GRID_WIDTH-1; ++i) {
    for (int j = 0; j < GRID_HEIGHT-1; ++j) {
      int node00 = i * GRID_HEIGHT + j; 
      int node01 = i * GRID_HEIGHT + j + 1; 
      int node10 = (i + 1) * GRID_HEIGHT + j; 
      int node11 = (i + 1) * GRID_HEIGHT + j + 1; 
      
      elements.add(new Element(nodes.get(node00), nodes.get(node11), nodes.get(node10)));
      elements.add(new Element(nodes.get(node00), nodes.get(node01), nodes.get(node11)));
    }
  }
}

void draw() {
  background(0);
  
  noStroke();
  rectMode(CENTER);
  
  for (int i = 0; i < 10; ++i) {
    stepSim(0.001f);
  }
  
  pushStyle();
    renderElements();
  popStyle();
}

void stepSim(float dt) {
  for (Element e : elements) {
    e.updateDispGrad();
    e.updateStrain();
    e.updateStress();
    e.updateEnergy();
    e.applyNodeForces();
  }
  
  PVector mpos = new PVector(mouseX, mouseY);
  for (Node n : nodes) {
    float floorViolation = max(n.worldPos.y - (float)SCR_HEIGHT + 100.0f, 0.0f);
    float leftViolation = max(-n.worldPos.x, 0.0f);
    float rightViolation = max(n.worldPos.x - (float)SCR_WIDTH, 0.0f);
    
    float KBarrier = 100000.0f;
    
    n.applyForce(new PVector((leftViolation - rightViolation) * KBarrier, GRAVITY * n.m - KBarrier * floorViolation));
    
    PVector diff = mpos.copy().sub(n.worldPos);
   
    float dist = diff.mag();
    diff.normalize();
   
    PVector mouseForce = new PVector(0.0f, 0.0f);
    if (mouseButton == LEFT && dist > 0.01 && dist < 1000) {
      float forceMag = min(500000000.0f / dist / dist, 10000000.0f);
      mouseForce = diff.mult(-forceMag);
    }
    
    n.applyForce(mouseForce);
  }
  
  for (Node n : nodes) {
    n.forwardEuler(dt);  
  } 
}

void renderElements() {
  stroke(0, 0, 255, 255);
  
  for (Element e : elements) {
    e.render(); 
  }
  
  //strokeWeight(4);
  //stroke(255, 0, 0, 255);  
  //for (Node n : nodes) {
  //  n.render();
  //}
}

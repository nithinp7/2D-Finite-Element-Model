class Node {
  PVector restPos;
  PVector worldPos;
  PVector worldVel;
  
  float m = 10.0f;
  
  // Current frame's force vector
  PVector force;
  
  Node(float restX, float restY) {
    restPos = new PVector(restX, restY);
    worldPos = new PVector(restX, restY);
    worldVel = new PVector(0.0f, 0.0f);
    force = new PVector(0.0f, 0.0f);
  }
  
  void forwardEuler(float dt) {
    force.add(worldVel.copy().mult(-0.01f));
    worldVel.add(force.copy().mult(dt / m));
    worldPos.add(worldVel.copy().mult(dt));
    force = new PVector(0.0f, 0.0f);
  }
  
  void applyForce(PVector df) {
    force.add(df);  
  }
  
  void render() {
    point(worldPos.x, worldPos.y);
  }
}

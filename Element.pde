import pallav.Matrix.*;

class Element {
  Node a;
  Node b;
  Node c;
  
  boolean fractured = false;
  
  // Transforms material coordinates to barycentric element coords
  float[][] Beta = new float[3][3];
  
  // Current state of the element
  float[][] dispGrad = new float[2][2];
  float[][] strain = new float[2][2];
  float[][] stress = new float[2][2];
  
  float elasticPotential;
  
  Element(Node a_, Node b_, Node c_) {
    a = a_;
    b = b_;
    c = c_;
    
    float[][] InvBeta = new float[][] {
      { this.a.restPos.x, this.b.restPos.x, this.c.restPos.x },
      { this.a.restPos.y, this.b.restPos.y, this.c.restPos.y },
      { 1.0f, 1.0f, 1.0f, 1.0f }
    };
    
    Beta = Matrix.inverse(InvBeta);
  }
  
  void updateDispGrad() {
    // Transforms barycentric coordinates to world pos (2x3)
    float[][] P = new float[][] {
      { this.a.worldPos.x, this.b.worldPos.x, this.c.worldPos.x },
      { this.a.worldPos.y, this.b.worldPos.y, this.c.worldPos.y }
    };
    
    // 2x3
    float[][] P_Beta = Matrix.Multiply(P, this.Beta);
    
    this.dispGrad[0][0] = P_Beta[0][0];
    //this.dispGrad[0][1] = P_Beta[1][0];
    //this.dispGrad[1][0] = P_Beta[0][1];
    this.dispGrad[1][1] = P_Beta[1][1];
    
    
    this.dispGrad[0][1] = P_Beta[0][1];
    this.dispGrad[1][0] = P_Beta[1][0];
  }
  
  void updateStrain() {
    float[][] I = Matrix.identity(2).array;
    
    // Green's strain tensor
    this.strain =
        Matrix.subtract(
            Matrix.Multiply(
              Matrix.transpose(this.dispGrad),
              this.dispGrad),
            I);
            
    // Matrix.Multiply can't take a float for some reason??
    this.strain[0][0] *= 0.5f;
    this.strain[0][1] *= 0.5f;
    this.strain[1][0] *= 0.5f;
    this.strain[1][1] *= 0.5f; 
  }
  
  // TODO: Allow YM and PR to vary per-element, instead of using global 
  // constants
  void updateStress() {
    this.stress[0][0] = C * (OMPR * strain[0][0] + PR * strain[1][1]);
    this.stress[1][1] = C * (PR * strain[0][0] + OMPR * strain[1][1]);
    this.stress[0][1] = this.stress[1][0] = C * OM2PR * strain[0][1];
    
    //float stressMag = sqrt(stress[0][0] * stress[0][0] + stress[0][1] * stress[0][1] * 4 + stress[1][1] * stress[1][1]);
    //if (stressMag > 100000000.0f) {
    // this.fractured = true; 
    //}
  }
  
  void updateEnergy() {
    this.elasticPotential = 0.5f * (
        this.stress[0][0] * this.strain[0][0] + 
        this.stress[0][1] * this.strain[0][1] + 
        this.stress[1][0] * this.strain[1][0] +
        this.stress[1][1] * this.strain[1][1]);
        
    if (this.elasticPotential > 10000000.0f) {
      this.fractured = true; 
    }
  }
  
  void applyNodeForces() {
    if (this.fractured) {
     return; 
    }
    
    // TODO: actually compute volume / area
    float vol = 1.0f;
    
    // TODO: explain this code with comments
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        float s = 0.0f;
        for (int k = 0; k < 2; ++k) {
          for (int l = 0; l < 2; ++l) {
            s -= Beta[j][l] * Beta[i][k] * stress[k][l];
          }
        }
        
        PVector f = nodes(j).worldPos.copy().mult(0.5f * s * vol);
        nodes(i).applyForce(f);
      }
    }
  }
  
  void render() {
    if (this.fractured) {
      return;
    }
    
    float stressMag = sqrt(
        stress[0][0] * stress[0][0] +
        stress[0][1] * stress[0][1] +
        stress[1][0] * stress[1][0] + 
        stress[1][1] * stress[1][1]);
        
    float col = stressMag / 150000.0f;
    stroke(col, 0.5f * col, 255 - col, 255.0f);
    
    line(a.worldPos.x, a.worldPos.y, b.worldPos.x, b.worldPos.y); 
    line(a.worldPos.x, a.worldPos.y, c.worldPos.x, c.worldPos.y); 
    line(c.worldPos.x, c.worldPos.y, b.worldPos.x, b.worldPos.y); 
  }
  
  Node nodes(int index) {
    if (index == 0) {
      return a; 
    } else if (index == 1) {
      return b; 
    } else if (index == 2) {
      return c; 
    } 
    
    return null;
  }
}

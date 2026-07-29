import { Vector3 } from 'three';

export class WindSystem {
  intensity: number = 0;
  baseDir: Vector3 = new Vector3(1, 0, 0);
  
  setIntensity(i: number) {
    this.intensity = i;
  }

  getWindForce(time: number): Vector3 {
    if (this.intensity === 0) return new Vector3(0,0,0);
    
    // Simple pseudo-random gust based on time
    const gustX = Math.sin(time * 2.1) * Math.cos(time * 0.8);
    const gustZ = Math.cos(time * 1.5) * Math.sin(time * 1.2);
    
    const force = new Vector3(
      this.baseDir.x * (1 + gustX * 0.5),
      (Math.sin(time * 3) * 0.2), // slight vertical drafts
      this.baseDir.z * (1 + gustZ * 0.5)
    );
    
    return force.multiplyScalar(this.intensity * 5); // Max 5N wind force
  }
}

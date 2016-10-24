import RigidBody from './RigidBody';

class MergedRigidBodyList {
  axis: number;
  bodies: RigidBody[];
  resting: RigidBody;

  constructor(axis: number) {
    this.axis = axis;
    this.bodies = [];
    this.resting = null;
  }

  add(body: RigidBody) {
    this.bodies.push(body);
  }

  getAverageVelocity() {
    // Total mass
    let totalMass = 0;
    let totalMassInfinity = 0;
    for (const body of this.bodies) {
      if (body.mass === Infinity) {
        totalMassInfinity += 1;
      } else {
        totalMass += body.mass;
      }
    }

    // Total momentum
    let totalMomentum = 0;
    let totalInfinityCoefficient = 0;
    for (const body of this.bodies) {
      if (body.mass === Infinity) {
        totalInfinityCoefficient += body.velocity[this.axis];
      } else {
        totalMomentum += body.mass * body.velocity[this.axis];
      }
    }

    if (totalMassInfinity) {
      return totalInfinityCoefficient / totalMassInfinity;
    } else {
      return totalMomentum / totalMass;
    }
  }

  reset() {
    this.bodies.length = 0;
    this.resting = null;
  }
}

export default MergedRigidBodyList;

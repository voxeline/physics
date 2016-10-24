import vec3 = require('gl-matrix/src/gl-matrix/vec3');
import AABB from './AABB';
import MergedRigidBodyList from './MergedRigidBodyList';

/*
 *    RIGID BODY - internal data structure
 *  Only AABB bodies right now. Someday will likely need spheres?
*/

export interface OnCollide {
  (impacts: vec3): any;
}

const zeros = vec3.create();

export interface RigidBodyOptions {
  onMove?: () => any;
}

class RigidBody {
  aabb: AABB;
  mass: number;
  friction: number;
  restitution: number;
  gravityMultiplier: number;
  onCollide: OnCollide;
  autoStep: boolean;
  onStep: Function;
  velocity: vec3;
  resting: vec3;
  inFluid: boolean;
  _forces: vec3;
  _impulses: vec3;
  _oldResting: vec3;
  _tmpBox: AABB;
  _dx: vec3;
  _merged: MergedRigidBodyList[];
  _out: vec3;

  constructor(
    position: vec3, size: vec3, mass: number, friction: number, restitution: number, gravMult: number,
    options?: RigidBodyOptions, onCollide?: OnCollide, autoStep?: boolean
  ) {
    this.aabb = new AABB(position, size, options.onMove); // clone
    this.mass = mass;

    // max friction force - i.e. friction coefficient times gravity
    this.friction = friction;
    this.restitution = restitution;
    this.gravityMultiplier = gravMult;
    this.onCollide = onCollide;
    this.autoStep = autoStep;
    this.onStep = null;

    // internals
    this.velocity = vec3.create();
    this.resting = vec3.fromValues(0, 0, 0);
    this.inFluid = false;
    this._forces = vec3.create();
    this._impulses = vec3.create();
    this._oldResting = vec3.create();
    this._tmpBox = new AABB(zeros, zeros);
    this._dx = vec3.create();
    this._merged = [null, null, null];
    this._out = vec3.create();
  }

  applyForce(f: vec3) {
    vec3.add(this._forces, this._forces, f);
  }

  applyImpulse(i: vec3) {
    vec3.add(this._impulses, this._impulses, i);
  }

  // temp
  atRestX() {
    return this.resting[0];
  }

  atRestY() {
    return this.resting[1];
  }

  atRestZ() {
    return this.resting[2];
  }
}

export default RigidBody;

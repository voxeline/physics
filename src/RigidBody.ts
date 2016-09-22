import vec3 = require('gl-matrix/src/gl-matrix/vec3');
import AABB from './AABB';

/*
 *    RIGID BODY - internal data structure
 *  Only AABB bodies right now. Someday will likely need spheres?
*/

export interface OnCollide {
  (impacts: vec3): any;
}

const v0 = vec3.create();

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

  constructor(
    _aabb: AABB, mass: number, friction: number, restitution: number, gravMult: number,
    onCollide?: OnCollide, autoStep?: boolean
  ) {
    this.aabb = new AABB(_aabb.base, _aabb.vec); // clone
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
  }

  setPosition(p: vec3) {
    vec3.subtract(v0, p, this.aabb.base);
    this.aabb.translate(v0);
  }

  getPosition() {
    return vec3.clone(this.aabb.base);
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

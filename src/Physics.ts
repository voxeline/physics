import vec3 = require('gl-matrix/src/gl-matrix/vec3');
import RigidBody, { OnCollide } from './RigidBody';
import objectAssign = require('object-assign');
import sweep, { TestVoxel } from './sweep';
import AABB from './AABB';

import {
  equals,
  cloneAABB,
} from './utils';

export interface PhysicsOptions {
  gravity?: vec3;
  airFriction?: number;
  minBounceImpulse?: number;
  fluidDensity?: number;
  fluidDrag?: number;
}

const tv0 = vec3.create();
const tv1 = vec3.create();

const _friction = vec3.create();
const a = vec3.create();
const g = vec3.create();
const dv = vec3.create();
const _dx = vec3.create();
const impacts = vec3.create();
const oldResting = vec3.create();

const defaults: PhysicsOptions = {
  gravity: vec3.fromValues(0, -10, 0),
  airFriction: 0.995,
  minBounceImpulse: .5, // lowest collision impulse that bounces
  fluidDensity: 1.2,
  fluidDrag: 4.0,
};

/*
 *    CONSTRUCTOR - represents a world of rigid bodies.
 *
 *  Takes testSolid(x,y,z) function to query block solidity
 *  Takes testFluid(x,y,z) function to query if a block is a fluid
*/
class Physics {
  gravity: vec3;
  airFriction: number;
  fluidDensity: number;
  fluidDrag: number;
  minBounceImpulse: number;
  bodies: RigidBody[];
  testSolid: TestVoxel;
  testFluid: TestVoxel;

  constructor(opts: PhysicsOptions, testSolid: TestVoxel, testFluid: TestVoxel) {
    const options = objectAssign({}, defaults, opts);

    this.gravity = options.gravity;
    this.airFriction = options.airFriction;
    this.fluidDensity = options.fluidDensity;
    this.fluidDrag = options.fluidDrag;
    this.minBounceImpulse = options.minBounceImpulse;
    this.bodies = [];

    // collision function - TODO: abstract this into a setter?
    this.testSolid = testSolid;
    this.testFluid = testFluid;
  }

  /*
  *    ADDING AND REMOVING RIGID BODIES
  */

  addBody(
    _aabb: AABB, mass?: number, friction?: number, restitution?: number,
    gravMult?: number, onCollide?: OnCollide
  ) {
    _aabb = _aabb || new AABB(vec3.set(tv0, 0, 0, 0), vec3.set(tv1, 1, 1, 1));

    if (typeof mass === 'undefined') mass = 1;
    if (typeof friction === 'undefined') friction = 1;
    if (typeof restitution === 'undefined') restitution = 0;
    if (typeof gravMult === 'undefined') gravMult = 1;
    const b = new RigidBody(_aabb, mass, friction, restitution, gravMult, onCollide);
    this.bodies.push(b);
    return b;
  }

  removeBody(b: RigidBody): void {
    const i = this.bodies.indexOf(b);
    if (i < 0) return undefined;
    this.bodies.splice(i, 1);
    b.aabb = b.onCollide = null; // in case it helps the GC
  }

  add(b: RigidBody): void {
    const i = this.bodies.indexOf(b);
    if (i >= 0) return;
    this.bodies.push(b);
  }

  remove(b: RigidBody): void {
    return this.removeBody(b);
  }

  /*
  *    PHYSICS AND COLLISIONS
  */

  tick (delta: number) {
    // convert dt to seconds
    const dt = delta / 1000;

    for (let i = 0, len = this.bodies.length; i < len; ++i) {
      const b = this.bodies[i];
      vec3.copy(oldResting, b.resting);

      // semi-implicit Euler integration

      // a = f/m + gravity*gravityMultiplier
      vec3.scale(a, b._forces, 1 / b.mass);
      vec3.scale(g, this.gravity, b.gravityMultiplier);
      vec3.add(a, a, g);

      // v1 = v0 + i/m + a*dt
      vec3.scale(dv, b._impulses, 1 / b.mass);
      vec3.add(b.velocity, b.velocity, dv);
      vec3.scale(dv, a, dt);
      vec3.add(b.velocity, b.velocity, dv);

      // apply friction if body was on ground last frame
      if (oldResting[1] < 0) {
        // friction force <= - u |vel|
        // max friction impulse = (F/m)*dt = (mg)/m*dt = u*g*dt = dt*b.friction
        const fMax = dt * b.friction;
        // friction direction - inversed horizontal velocity
        vec3.scale(_friction, b.velocity, -1);
        _friction[1] = 0;
        const vAmt = vec3.length(_friction);
        if (vAmt > fMax) { // slow down
          vec3.scale(_friction, _friction, fMax / vAmt);
          vec3.add(b.velocity, b.velocity, _friction);
        } else { // stop
          b.velocity[0] = b.velocity[2] = 0;
        }
      } else {
        // not on ground, apply air resistance
        vec3.scale(b.velocity, b.velocity, this.airFriction);
      }

      // x1-x0 = v1*dt
      vec3.scale(_dx, b.velocity, dt);

      // clear forces and impulses for next timestep
      vec3.set(b._forces, 0, 0, 0);
      vec3.set(b._impulses, 0, 0, 0);

      // cache old position for use in autostepping
      if (b.autoStep) {
        cloneAABB(tmpBox, b.aabb);
      }

      // sweeps aabb along dx and accounts for collisions
      processCollisions(this, b.aabb, _dx, b.resting);

      // if autostep, and on ground, run collisions again with stepped up aabb
      if (b.autoStep) {
        tryAutoStepping(this, b, tmpBox, _dx);
      }

      // Collision impacts. b.resting shows which axes had collisions:
      for (let j = 0; j < 3; ++j) {
        impacts[j] = 0;
        if (b.resting[j]) {
          // count impact only if wasn't collided last frame
          if (!oldResting[j]) impacts[j] = -b.velocity[j];
          b.velocity[j] = 0;
        }
      }
      const mag = vec3.length(impacts);
      if (mag > .001) { // epsilon
        // bounce if over minBounceImpulse
        if (mag > this.minBounceImpulse && b.restitution) {
          vec3.scale(impacts, impacts, b.restitution * b.mass);
          b.applyImpulse(impacts);
        }
        // send collision event regardless
        if (b.onCollide) b.onCollide(impacts);
      }

      // First pass at handling fluids. Assumes fluids are settled
      //   thus, only check at center of body, and only from bottom up
      const box = b.aabb;
      const cx = Math.floor((box.base[0] + box.max[0]) / 2);
      const cz = Math.floor((box.base[2] + box.max[2]) / 2);
      const y0 = Math.floor(box.base[1]);
      const y1 = Math.floor(box.max[1]);
      let submerged = 0;
      for (let cy = y0; cy <= y1; ++cy) {
        if (this.testFluid(cx, cy, cz)) {
          ++submerged;
        } else {
          break;
        }
      }

      if (submerged > 0) {
        // find how much of body is submerged
        const fluidLevel = y0 + submerged;
        const heightInFluid = fluidLevel - box.base[1];
        let ratioInFluid = heightInFluid / box.size[1];
        if (ratioInFluid > 1) ratioInFluid = 1;
        const vol = box.size[0] * box.size[1] * box.size[2];
        const displaced = vol * ratioInFluid;
        // bouyant force = -gravity * fluidDensity * volumeDisplaced
        vec3.scale(g, this.gravity, -b.gravityMultiplier * this.fluidDensity * displaced);
        // drag force = -dv for some constant d. Here scale it down by ratioInFluid
        vec3.scale(_friction, b.velocity, -this.fluidDrag * ratioInFluid);
        vec3.add(g, g, _friction);
        b.applyForce(g);
        b.inFluid = true;
      } else {
        b.inFluid = false;
      }

    }
  }
}

// main collision processor - sweep aabb along velocity vector and set resting vector
function processCollisions(self: Physics, box: AABB, velocity: vec3, resting: vec3) {
  vec3.set(resting, 0, 0, 0);
  sweep(tv0, self.testSolid, box, velocity, function (dist, axis, dir, vec) {
    resting[axis] = dir;
    vec[axis] = 0;
  });
  box.translate(tv0);
}

const tmpBox = new AABB(vec3.set(tv0, 0, 0, 0), vec3.set(tv1, 0, 0, 0));
const tmpResting = vec3.create();
const targetPos = vec3.create();
const upvec = vec3.create();
const leftover = vec3.create();

function tryAutoStepping(self: Physics, b: RigidBody, oldBox: AABB, dx: vec3) {
  if (b.resting[1] >= 0 && !b.inFluid) return;

  // // direction movement was blocked before trying a step
  const xBlocked = (b.resting[0] !== 0);
  const zBlocked = (b.resting[2] !== 0);
  if (!(xBlocked || zBlocked)) return;

  // continue autostepping only if headed sufficiently into obstruction
  const ratio = Math.abs(dx[0] / dx[2]);
  const cutoff = 4;
  if (!xBlocked && ratio > cutoff) return;
  if (!zBlocked && ratio < 1 / cutoff) return;

  // original target position before being obstructed
  vec3.add(targetPos, oldBox.base, dx);

  // move towards the target until the first X/Z collision
  const getVoxels = self.testSolid;
  sweep(tv0, getVoxels, oldBox, dx, function (dist, axis, dir, vec) {
    if (axis === 1) {
      vec[axis] = 0;
    } else {
      return true;
    }
  });
  oldBox.translate(tv0);

  const y = b.aabb.base[1];
  const ydist = Math.floor(y + 1.001) - y;
  vec3.set(upvec, 0, ydist, 0);
  let collided = false;
  // sweep up, bailing on any obstruction
  sweep(tv0, getVoxels, oldBox, upvec, function (dist, axis, dir, vec) {
    collided = true;
    return true;
  });
  oldBox.translate(tv0);
  if (collided) return; // could't move upwards

  // now move in X/Z however far was left over before hitting the obstruction
  vec3.subtract(leftover, targetPos, oldBox.base);
  leftover[1] = 0;
  processCollisions(self, oldBox, leftover, tmpResting);

  // bail if no movement happened in the originally blocked direction
  if (xBlocked && !equals(oldBox.base[0], targetPos[0])) return;
  if (zBlocked && !equals(oldBox.base[2], targetPos[2])) return;

  // done - oldBox is now at the target autostepped position
  cloneAABB(b.aabb, oldBox);
  b.resting[0] = tmpResting[0];
  b.resting[2] = tmpResting[2];
  if (b.onStep) b.onStep();
}

export default Physics;

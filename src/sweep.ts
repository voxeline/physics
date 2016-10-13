import vec3 = require('gl-matrix/src/gl-matrix/vec3');
import AABB from './AABB';

export interface TestVoxel {
  (x: number, y: number, z: number): boolean;
}

export interface SweepCallback {
  (cumulativeT: number, axis: number, dir: number, left: vec3): any;
}

const tr_arr = vec3.create();
const ldi_arr = vec3.create();
const tri_arr = vec3.create();
const step_arr = vec3.create();
const tDelta_arr = vec3.create();
const tNext_arr = vec3.create();
const vec_arr = vec3.create();
const normed_arr = vec3.create();
const base_arr = vec3.create();
const max_arr = vec3.create();
const left_arr = vec3.create();
const result_arr = vec3.create();

const v0 = vec3.create();

// core implementation:

function sweep_impl(getVoxel: TestVoxel, callback: SweepCallback, vec: vec3, base: vec3, max: vec3, epsilon: number) {

  // consider algo as a raycast along the AABB's leading corner
  // as raycast enters each new voxel, iterate in 2D over the AABB's
  // leading face in that axis looking for collisions
  //
  // original raycast implementation: https://github.com/andyhall/fast-voxel-raycast
  // original raycast paper: http://www.cse.chalmers.se/edu/year/2010/course/TDA361/grid.pdf

  const tr = tr_arr;
  const ldi = ldi_arr;
  const tri = tri_arr;
  const step = step_arr;
  const tDelta = tDelta_arr;
  const tNext = tNext_arr;
  const normed = normed_arr;

  let cumulative_t = 0.0;
  let t = 0.0;
  let max_t = 0.0;
  let axis = 0;

  // init for the current sweep vector and take first step
  initSweep();
  if (max_t === 0) return 0;

  axis = stepForward();

  // loop along raycast vector
  while (t <= max_t) {
    // sweeps over leading face of AABB
    if (checkCollision(axis)) {
      // calls the callback and decides whether to continue
      const done = handleCollision();
      if (done) return cumulative_t;
    }

    axis = stepForward();
  }

  // reached the end of the vector unobstructed, finish and exit
  cumulative_t += max_t;
  vec3.add(base, base, vec);
  vec3.add(max, max, vec);

  return cumulative_t;

  // low-level implementations of each step:
  function initSweep() {
    // parametrization t along raycast
    t = 0.0;
    max_t = Math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    if (max_t === 0) return;
    for (let i = 0; i < 3; i++) {
      const dir = (vec[i] >= 0);
      step[i] = dir ? 1 : -1;
      // trailing / trailing edge coords
      const lead = dir ? max[i] : base[i];
      tr[i] = dir ? base[i] : max[i];
      // int values of lead/trail edges
      ldi[i] = leadEdgeToInt(lead, step[i]);
      tri[i] = trailEdgeToInt(tr[i], step[i]);
      // normed vector
      normed[i] = vec[i] / max_t;
      // distance along t required to move one voxel in each axis
      tDelta[i] = Math.abs(1 / normed[i]);
      // location of nearest voxel boundary, in units of t
      const dist = dir ? (ldi[i] + 1 - lead) : (lead - ldi[i]);
      tNext[i] = (tDelta[i] < Infinity) ? tDelta[i] * dist : Infinity;
    }
  }


  // check for collisions - iterate over the leading face on the advancing axis

  function checkCollision(i_axis: number) {
    const stepx = step[0];
    const x0 = (i_axis === 0) ? ldi[0] : tri[0];
    const x1 = ldi[0] + stepx;

    const stepy = step[1];
    const y0 = (i_axis === 1) ? ldi[1] : tri[1];
    const y1 = ldi[1] + stepy;

    const stepz = step[2];
    const z0 = (i_axis === 2) ? ldi[2] : tri[2];
    const z1 = ldi[2] + stepz;

    // const j_axis = (i_axis + 1) % 3
    // const k_axis = (i_axis + 2) % 3
    // const s = ['x', 'y', 'z'][i_axis]
    // const js = ['x', 'y', 'z'][j_axis]
    // const ks = ['x', 'y', 'z'][k_axis]
    // const i0 = [x0, y0, z0][i_axis]
    // const j0 = [x0, y0, z0][j_axis]
    // const k0 = [x0, y0, z0][k_axis]
    // const i1 = [x1 - stepx, y1 - stepy, z1 - stepz][i_axis]
    // const j1 = [x1 - stepx, y1 - stepy, z1 - stepz][j_axis]
    // const k1 = [x1 - stepx, y1 - stepy, z1 - stepz][k_axis]
    // console.log('=== step', s, 'to', i0, '   sweep', js, j0 + ',' + j1, '   ', ks, k0 + ',' + k1)

    for (let x = x0; x != x1; x += stepx) {
      for (let y = y0; y != y1; y += stepy) {
        for (let z = z0; z != z1; z += stepz) {
          if (getVoxel(x, y, z)) return true;
        }
      }
    }
    return false;
  }


  // on collision - call the callback and return or set up for the next sweep

  function handleCollision() {

    // set up for callback
    cumulative_t += t;
    const dir = step[axis];

    // vector moved so far, and left to move
    const done = t / max_t;
    const left = left_arr;

    vec3.scale(v0, vec, done);
    vec3.add(base, base, v0);
    vec3.add(max, max, v0);
    vec3.sub(left, vec, v0);

    // set leading edge of stepped axis exactly to voxel boundary
    // else we'll sometimes rounding error beyond it
    if (dir > 0) {
      max[axis] = Math.round(max[axis]);
    } else {
      base[axis] = Math.round(base[axis]);
    }

    // call back to let client update the "left to go" vector
    const res = callback(cumulative_t, axis, dir, left);

    // bail out out on truthy response
    if (res) return true;

    // init for new sweep along vec
    vec3.copy(vec, left);
    initSweep();
    if (max_t === 0) return true; // no vector left

    return false;
  }


  // advance to next voxel boundary, and return which axis was stepped

  function stepForward() {
    const axis = (tNext[0] < tNext[1]) ?
        ((tNext[0] < tNext[2]) ? 0 : 2) :
        ((tNext[1] < tNext[2]) ? 1 : 2);
    const dt = tNext[axis] - t;
    t = tNext[axis];
    ldi[axis] += step[axis];
    tNext[axis] += tDelta[axis];
    for (let i = 0; i < 3; i++) {
      tr[i] += dt * normed[i];
      tri[i] = trailEdgeToInt(tr[i], step[i]);
    }

    return axis;
  }

  function leadEdgeToInt(coord: number, step: number) {
    return Math.floor(coord - step * epsilon);
  }

  function trailEdgeToInt(coord: number, step: number) {
    return Math.floor(coord + step * epsilon);
  }
}

// conform inputs

function sweep(getVoxel: TestVoxel, box: AABB, dir: vec3, callback: SweepCallback, noTranslate?: boolean, epsilon?: number) {
  const vec = vec_arr;
  const base = base_arr;
  const max = max_arr;
  const result = result_arr;

  // init parameter float arrays
  vec3.copy(vec, dir);
  vec3.copy(max, box.max);
  vec3.copy(base, box.base);

  if (!epsilon) epsilon = 1e-10;

  // run sweep implementation
  const dist = sweep_impl(getVoxel, callback, vec, base, max, epsilon);

  // translate box by distance needed to updated base value
  if (!noTranslate) {
    for (let i = 0; i < 3; i++) {
      result[i] = (dir[i] > 0) ? max[i] - box.max[i] : base[i] - box.base[i];
    }
    box.translate(result);
  }

  // return value is total distance moved (not necessarily magnitude of [end]-[start])
  return dist;
}

export default sweep;

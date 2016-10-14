import vec3 = require('gl-matrix/src/gl-matrix/vec3');
import AABB from './AABB';

export function equals(a: number, b: number) {
  return Math.abs(a - b) < 1e-5;
}

export function cloneAABB(tgt: AABB, src: AABB) {
  vec3.copy(tgt.base, src.base);
  vec3.copy(tgt.max, src.max);
  vec3.copy(tgt.size, src.size);
  vec3.copy(tgt.area, src.area);
}

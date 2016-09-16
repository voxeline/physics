import AABB from './AABB';

export function equals(a: number, b: number) {
  return Math.abs(a - b) < 1e-5;
}

export function cloneAABB(tgt: AABB, src: AABB) {
  for (let i = 0; i < 3; i++) {
    tgt.base[i] = src.base[i];
    tgt.max[i] = src.max[i];
    tgt.vec[i] = src.vec[i];
  }
}

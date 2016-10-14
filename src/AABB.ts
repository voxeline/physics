import vec3 = require('gl-matrix/src/gl-matrix/vec3');

const v0 = vec3.create();
const v1 = vec3.create();

class AABB {
  size: vec3;
  base: vec3;
  center: vec3;
  max: vec3;

  // Surface area
  area: vec3;

  constructor(center: vec3, size: vec3) {
    this.size = vec3.create();
    this.base = vec3.create();
    this.center = vec3.create();
    this.max = vec3.create();
    this.area = vec3.create();

    this.setCenterSize(center, size);
  }

  setCenterSize(center: vec3, size: vec3) {
    vec3.copy(this.size, size);
    vec3.set(this.area,
      size[1] * size[2],
      size[2] * size[0],
      size[0] * size[1]
    );

    return this.setCenter(center);
  }

  setCenter(center: vec3) {
    vec3.copy(this.center, center);

    const sizeHalf = vec3.scale(v0, this.size, 0.5);
    vec3.sub(this.base, center, sizeHalf);
    vec3.add(this.max, center, sizeHalf);
    return this;
  }

  width() {
    return this.size[0];
  }

  height() {
    return this.size[1];
  }

  depth() {
    return this.size[2];
  }

  x0() {
    return this.base[0];
  }

  y0() {
    return this.base[1];
  }

  z0() {
    return this.base[2];
  }

  x1() {
    return this.max[0];
  }

  y1() {
    return this.max[1];
  }

  z1() {
    return this.max[2];
  }

  translate(by: vec3) {
    vec3.add(this.max, this.max, by);
    vec3.add(this.center, this.center, by);
    vec3.add(this.base, this.base, by);
    return this;
  }

  expand(aabb: AABB) {
    const max = vec3.create();
    const min = vec3.create();

    vec3.max(max, aabb.max, this.max);
    vec3.min(min, aabb.base, this.base);
    vec3.subtract(max, max, min);

    return new AABB(min, max);
  }

  intersects(aabb: AABB) {
    if (aabb.base[0] > this.max[0]) return false;
    if (aabb.base[1] > this.max[1]) return false;
    if (aabb.base[2] > this.max[2]) return false;
    if (aabb.max[0] < this.base[0]) return false;
    if (aabb.max[1] < this.base[1]) return false;
    if (aabb.max[2] < this.base[2]) return false;

    return true;
  }

  touches(aabb: AABB) {
    const intersection = this.union(aabb);

    return (intersection !== null) &&
          ((intersection.width() === 0) ||
          (intersection.height() === 0) ||
          (intersection.depth() === 0));
  }

  union(aabb: AABB) {
    if (!this.intersects(aabb)) return null;

    const baseX = Math.max(aabb.base[0], this.base[0]);
    const baseY = Math.max(aabb.base[1], this.base[1]);
    const baseZ = Math.max(aabb.base[2], this.base[2]);
    const maxX = Math.min(aabb.max[0], this.max[0]);
    const maxY = Math.min(aabb.max[1], this.max[1]);
    const maxZ = Math.min(aabb.max[2], this.max[2]);

    return new AABB(
      vec3.set(v0, baseX, baseY, baseZ),
      vec3.set(v1, maxX - baseX, maxY - baseY, maxZ - baseZ)
    );
  }

  unionSize(out: vec3, aabb: AABB) {
    const min = vec3.max(v0, aabb.base, this.base);
    const max = vec3.min(v1, aabb.max, this.max);
    return vec3.sub(out, max, min);
  }
}

export default AABB;

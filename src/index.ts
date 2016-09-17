import Physics, {
  PhysicsOptions,
  TestVoxel,
} from './Physics';

import RigidBody from './RigidBody';
import AABB from './AABB';

export {
  Physics,
  RigidBody,
  AABB,
}

export default function createPhysics(opts: PhysicsOptions, testSolid: TestVoxel, testFluid: TestVoxel) {
  return new Physics(opts, testSolid, testFluid);
}

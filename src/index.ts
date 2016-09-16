import Physics, {
  PhysicsOptions,
  TestVoxel,
} from './Physics';

export default function createPhysics(opts: PhysicsOptions, testSolid: TestVoxel, testFluid: TestVoxel) {
  return new Physics(opts, testSolid, testFluid);
}

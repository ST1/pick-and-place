"""
Viam Pick-and-Place Workshop — Starter Script
==============================================

Copy your connection details from the machine's Connect tab -> Python SDK.
Fill in the TODOs in order. Run after each section to verify before moving on.

    uv run python starter-script.py        # recommended (uv reads ../scripts/pyproject.toml)
    # or, without uv:
    python3 starter-script.py

Prerequisites:
    python3 --version                                # must be 3.10+
    uv add viam-sdk                                  # or: pip install viam-sdk
    uv run python -c "import viam; print(viam.__version__)"
"""

import asyncio
import os

from viam.robot.client import RobotClient
from viam.components.gripper import Gripper
from viam.components.switch import Switch
from viam.services.motion import MotionClient
from viam.services.vision import VisionClient
from viam.proto.common import PoseInFrame, Pose

# --- Tuning constants ---------------------------------------------------------
GRIPPER_LENGTH_MM = (
    -60
)  # offset from the gripper's claw-geometry TCP to the real fingertip contact point
APPROACH_MM = -100  # clearance above the block top before descending
MAX_BLOCK_MM = 120  # ignore detections larger than a block (tape, table edges)


def offset_pose(pose: Pose, z_offset_mm: float) -> Pose:
    """Raise or lower a pose in z while keeping x/y/orientation fixed."""
    return Pose(
        x=pose.x,
        y=pose.y,
        z=pose.z + z_offset_mm,
        o_x=pose.o_x,
        o_y=pose.o_y,
        o_z=pose.o_z,
        theta=pose.theta,
    )


# --- TODO 1: paste address + API key from the Connect tab ---------------------
# Read from env vars, or from scripts/.env (gitignored) so the key never lands in git.
_env = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
if os.path.exists(_env):
    for _line in open(_env):
        if "=" in _line and not _line.lstrip().startswith("#"):
            _k, _v = _line.strip().split("=", 1)
            os.environ.setdefault(_k, _v.strip().strip('"'))
MACHINE_ADDRESS = os.environ["VIAM_MACHINE_ADDRESS"]
API_KEY = os.environ["VIAM_API_KEY"]
API_KEY_ID = os.environ["VIAM_API_KEY_ID"]

# --- Resource names (must match the CONFIGURE tab exactly) --------------------
# armfarm14 uses the fragment's names, not the tutorial's *-1 names.
ARM_NAME = "arm"
GRIPPER_NAME = "gripper"
CAMERA_NAME = "cam"
VISION_NAME = "vision-segment"
HOME_POSE = "home-pose"
# approach/grasp come from perception in Phase 5; home doubles as travel.
PLACE_POSE = "place-pose"


async def connect() -> RobotClient:
    opts = RobotClient.Options.with_api_key(
        api_key=API_KEY,
        api_key_id=API_KEY_ID,
    )
    return await RobotClient.at_address(MACHINE_ADDRESS, opts)


async def main() -> None:
    async with await connect() as machine:
        # TODO 2: confirm the connection — list every resource on the machine.
        # You should see arm-1, gripper-1, cam-1, the poses as Switches,
        # and the obstacles as grippers.
        print(machine.resource_names)

        # TODO 3: get typed resource handles.
        gripper = Gripper.from_robot(machine, GRIPPER_NAME)

        home = Switch.from_robot(machine, HOME_POSE)
        place_pose = Switch.from_robot(machine, PLACE_POSE)

        motion = MotionClient.from_robot(machine, "builtin")
        vision = VisionClient.from_robot(machine, VISION_NAME)

        # TODO 4 (static sequence) skipped: approach/grasp/travel poses were not
        # saved on armfarm14; perception supplies approach/grasp below.

        async def step(label: str) -> None:
            if not PICK:
                return
            input(f"[Enter] {label}   (Ctrl+C to abort) ")

        # TODO 5: perception. Detect from home — the camera is wrist-mounted,
        # so its frame moves with the arm.
        await step("go to home-pose")
        if PICK:
            await home.set_position(2)
        objects = await vision.get_object_point_clouds(CAMERA_NAME)
        if not objects:
            print("No objects detected")
            return
        for o in objects:
            g = o.geometries.geometries[0]
            c = g.center
            d = g.box.dims_mm if g.HasField("box") else None
            dims = f"  box={d.x:.0f}x{d.y:.0f}x{d.z:.0f}mm" if d else ""
            print(f"  {g.label or '?':18s} cam x={c.x:7.1f} y={c.y:7.1f} z={c.z:7.1f}  points={len(o.point_cloud)}{dims}")
        # Drop anything too big to be a block (e.g. the blue tape square),
        # then keep only the requested color.
        def size_mm(o):
            g = o.geometries.geometries[0]
            return max(g.box.dims_mm.x, g.box.dims_mm.y, g.box.dims_mm.z) if g.HasField("box") else 0

        candidates = [o for o in objects if size_mm(o) <= MAX_BLOCK_MM]
        if COLOR:
            candidates = [o for o in candidates if o.geometries.geometries[0].label.endswith(COLOR)]
        if not candidates:
            print(f"No block matching color={COLOR or 'any'} under {MAX_BLOCK_MM} mm")
            return
        obj = max(candidates, key=lambda o: len(o.point_cloud))
        geometry = obj.geometries.geometries[0]
        print(f"Target: {geometry.label}")
        obj_in_cam = PoseInFrame(reference_frame=CAMERA_NAME, pose=geometry.center)

        # TODO 6: approach standoff above the block, then the remaining descent
        # straight down in the gripper's own frame.
        approach_pose = offset_pose(obj_in_cam.pose, APPROACH_MM)
        grasp_distance = abs(APPROACH_MM - GRIPPER_LENGTH_MM)
        world = await machine.transform_pose(obj_in_cam, "world")
        print(f"Target in world: x={world.pose.x:.1f} y={world.pose.y:.1f} z={world.pose.z:.1f} mm; "
              f"descent {grasp_distance} mm")

        if not PICK:
            print("Detect-only run. Re-run with --pick to move the arm.")
            return

        # TODO 7: pick with motion.move, place with the saved switch.
        # home-pose doubles as the travel pose.
        await step("move above the block")
        await motion.move(
            component_name=GRIPPER_NAME,
            destination=PoseInFrame(reference_frame=CAMERA_NAME, pose=approach_pose),
        )
        await gripper.open()

        await step(f"descend {grasp_distance} mm and grab")
        await motion.move(
            component_name=GRIPPER_NAME,
            destination=PoseInFrame(
                reference_frame=GRIPPER_NAME,
                pose=Pose(x=0, y=0, z=grasp_distance, o_x=0, o_y=0, o_z=1, theta=0),
            ),
        )
        await gripper.grab()
        await asyncio.sleep(0.3)

        await step("lift to home-pose")
        await home.set_position(2)
        await step("go to place-pose")
        await place_pose.set_position(2)
        await gripper.open()
        await step("return to home-pose")
        await home.set_position(2)
        print("Pick and place complete")

if __name__ == "__main__":
    import sys

    PICK = "--pick" in sys.argv
    # --color red|green|blue|orange  (the yellow block is labeled "orange")
    COLOR = sys.argv[sys.argv.index("--color") + 1] if "--color" in sys.argv else None
    COLOR = {"yellow": "orange"}.get(COLOR, COLOR)
    asyncio.run(main())

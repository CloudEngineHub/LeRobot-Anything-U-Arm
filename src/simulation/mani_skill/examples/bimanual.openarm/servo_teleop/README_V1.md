# OpenArm v1 MuJoCo servo teleoperation

This is the OpenArm v1-specific entry point. It leaves the existing v2 script
unchanged.

Run from Windows:

```powershell
D:\openarm\servo_teleop\run_teleop_v1.ps1
```

The program releases all 16 leader servos, samples their zero pose for two
seconds, and maps angle changes to the v1 model. The 14 arm joints use clipped
PD torque control. Channel 8 on each leader controls both gripper slide joints.

Channel assignment and direction are configured explicitly in
`joint_mapping_v1.json`. Every OpenArm target joint has a `servo_id` from 0 to
7 and a `sign` of `1` or `-1`. Change `servo_id` when the wrong leader joint
drives a target joint. Change `sign` when the correct joint moves in the wrong
direction. Each servo ID must be used exactly once per arm, including the
gripper entry.

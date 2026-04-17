#!/usr/bin/env python3
"""
Generate plain URDF files from KUKA xacro macros and write a hand-crafted UR10e URDF.

Run from the project root:
    python3 scripts/generate_urdfs.py
"""

import os
import re

ROBOTS_DIR = os.path.join(os.path.dirname(__file__), "..", "assets", "robots")

# ── KUKA families ────────────────────────────────────────────────────────────
# Each entry: (family_folder, macro_file_stem, robot_name)
KUKA_ROBOTS = [
    # agilus
    ("kuka_agilus", "kr4_r600",       "kr4_r600"),
    ("kuka_agilus", "kr6_r700_2",     "kr6_r700_2"),
    ("kuka_agilus", "kr6_r700_sixx",  "kr6_r700_sixx"),
    ("kuka_agilus", "kr6_r900_2",     "kr6_r900_2"),
    ("kuka_agilus", "kr6_r900_sixx",  "kr6_r900_sixx"),
    ("kuka_agilus", "kr10_r900_2",    "kr10_r900_2"),
    ("kuka_agilus", "kr10_r1100_2",   "kr10_r1100_2"),
    # cybertech
    ("kuka_cybertech", "kr8_r1440_2_arc_hw",  "kr8_r1440_2"),
    ("kuka_cybertech", "kr8_r2100_2_arc_hw",  "kr8_r2100_2"),
    ("kuka_cybertech", "kr12_r1450_3_hw",     "kr12_r1450_3"),
    ("kuka_cybertech", "kr16_r1610_2",        "kr16_r1610_2"),
    ("kuka_cybertech", "kr16_r2010_2",        "kr16_r2010_2"),
    ("kuka_cybertech", "kr20_r1810_2",        "kr20_r1810_2"),
    # iontec
    ("kuka_iontec", "kr20_r3100", "kr20_r3100"),
    ("kuka_iontec", "kr30_r2100", "kr30_r2100"),
    ("kuka_iontec", "kr50_r2500", "kr50_r2500"),
    ("kuka_iontec", "kr70_r2100", "kr70_r2100"),
    # quantec
    ("kuka_quantec", "kr150_r3100_2",      "kr150_r3100_2"),
    ("kuka_quantec", "kr210_r2700_2",      "kr210_r2700_2"),
    ("kuka_quantec", "kr210_r3100_2",      "kr210_r3100_2"),
    ("kuka_quantec", "kr210_r3100_ultra",  "kr210_r3100_ultra"),
    ("kuka_quantec", "kr210_r3300_2_k",    "kr210_r3300_2_k"),
    ("kuka_quantec", "kr240_r2900_2",      "kr240_r2900_2"),
    ("kuka_quantec", "kr300_r2700_2",      "kr300_r2700_2"),
]


def generate_kuka_urdf(family: str, macro_stem: str, robot_name: str) -> bool:
    macro_path = os.path.join(ROBOTS_DIR, family, "urdf", f"{macro_stem}_macro.xacro")
    if not os.path.exists(macro_path):
        print(f"  [skip] macro not found: {macro_path}")
        return False

    with open(macro_path) as f:
        content = f.read()

    # 1. Strip prefix variables (we use empty prefix)
    content = content.replace("${prefix}", "")

    # 2. Replace all package:// URIs with relative paths
    #    package://ANYTHING/meshes/ → ../meshes/
    content = re.sub(r"package://[^/]+/", "../", content)

    # 3. Remove <inertial> blocks (not needed for rendering)
    content = re.sub(r"\s*<inertial>.*?</inertial>", "", content, flags=re.DOTALL)

    # 4. Extract the body of the xacro:macro element
    m = re.search(r"<xacro:macro[^>]*>(.*?)</xacro:macro>", content, re.DOTALL)
    if not m:
        print(f"  [skip] no xacro:macro found in {macro_path}")
        return False
    body = m.group(1)

    urdf = f"""<?xml version="1.0"?>
<!-- Auto-generated from {os.path.basename(macro_path)} -->
<robot name="{robot_name}">

  <link name="world"/>

  <joint name="world-base_link" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
{body}
</robot>
"""

    out_path = os.path.join(ROBOTS_DIR, family, "urdf", f"{robot_name}.urdf")
    with open(out_path, "w") as f:
        f.write(urdf)
    print(f"  wrote {out_path}")
    return True


# ── UR10e — hand-written URDF ────────────────────────────────────────────────
# Kinematics from Universal Robots UR10e specification:
#   d1=0.1273  shoulder_y_offset=0.1761
#   a2=-0.6120  a3=-0.5723
#   d4=0.1639  d5=0.1157  d6=0.0922
# Visual meshes: relative paths, DAE (URDFLoader applies -PI/2 X rotation to DAE geometry)
# Visual origins below match the ur_description package conventions.

UR10E_URDF = """\
<?xml version="1.0"?>
<!-- Hand-crafted UR10e URDF for CellGen.
     Kinematics: UR10e datasheet (d1=0.1273, a2=-0.6120, a3=-0.5723,
                  d4=0.1639, d5=0.1157, d6=0.0922).
     Meshes: ur_description DAE visuals + STL collision. -->
<robot name="ur10e">

  <link name="world"/>

  <link name="base_link">
    <visual>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
      <geometry>
        <mesh filename="../meshes/ur10e/visual/base.dae"/>
      </geometry>
      <material name="ur_grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0"/>
      <geometry>
        <mesh filename="../meshes/ur10e/collision/base.stl"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>

  <!-- ── Link: upper_arm (carries shoulder mesh, rotates with joint 1) ── -->
  <link name="upper_arm_link">
    <visual>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.176"/>
      <geometry>
        <mesh filename="../meshes/ur10e/visual/shoulder.dae"/>
      </geometry>
      <material name="ur_grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.176"/>
      <geometry>
        <mesh filename="../meshes/ur10e/collision/shoulder.stl"/>
      </geometry>
    </collision>
  </link>

  <joint name="shoulder_pan_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 0.1273"/>
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="330" lower="-6.283185307179586" upper="6.283185307179586" velocity="2.0943951023931953"/>
  </joint>

  <!-- ── Link: forearm (carries upper-arm mesh) ── -->
  <link name="forearm_link">
    <visual>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
      <geometry>
        <mesh filename="../meshes/ur10e/visual/upperarm.dae"/>
      </geometry>
      <material name="ur_grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
      <geometry>
        <mesh filename="../meshes/ur10e/collision/upperarm.stl"/>
      </geometry>
    </collision>
  </link>

  <joint name="shoulder_lift_joint" type="revolute">
    <origin rpy="0 1.5707963267948966 0" xyz="0 0.1761 0"/>
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="330" lower="-6.283185307179586" upper="6.283185307179586" velocity="2.0943951023931953"/>
  </joint>

  <!-- ── Link: wrist_1 (carries forearm mesh) ── -->
  <link name="wrist_1_link">
    <visual>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 -0.137"/>
      <geometry>
        <mesh filename="../meshes/ur10e/visual/forearm.dae"/>
      </geometry>
      <material name="ur_grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 -0.137"/>
      <geometry>
        <mesh filename="../meshes/ur10e/collision/forearm.stl"/>
      </geometry>
    </collision>
  </link>

  <joint name="elbow_joint" type="revolute">
    <origin rpy="0 0 0" xyz="-0.6120 0 0"/>
    <parent link="forearm_link"/>
    <child link="wrist_1_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="150" lower="-3.141592653589793" upper="3.141592653589793" velocity="3.141592653589793"/>
  </joint>

  <!-- ── Link: wrist_2 (carries wrist1 mesh) ── -->
  <link name="wrist_2_link">
    <visual>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0127"/>
      <geometry>
        <mesh filename="../meshes/ur10e/visual/wrist1.dae"/>
      </geometry>
      <material name="ur_grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0127"/>
      <geometry>
        <mesh filename="../meshes/ur10e/collision/wrist1.stl"/>
      </geometry>
    </collision>
  </link>

  <joint name="wrist_1_joint" type="revolute">
    <origin rpy="0 1.5707963267948966 0" xyz="-0.5723 0 0.1149"/>
    <parent link="wrist_1_link"/>
    <child link="wrist_2_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="54" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/>
  </joint>

  <!-- ── Link: wrist_3 (carries wrist2 mesh) ── -->
  <link name="wrist_3_link">
    <visual>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0127"/>
      <geometry>
        <mesh filename="../meshes/ur10e/visual/wrist2.dae"/>
      </geometry>
      <material name="ur_grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0127"/>
      <geometry>
        <mesh filename="../meshes/ur10e/collision/wrist2.stl"/>
      </geometry>
    </collision>
  </link>

  <joint name="wrist_2_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0.1157 0"/>
    <parent link="wrist_2_link"/>
    <child link="wrist_3_link"/>
    <axis xyz="0 0 1"/>
    <limit effort="54" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/>
  </joint>

  <!-- ── Link: tool0 (carries wrist3 mesh) ── -->
  <link name="tool0">
    <visual>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0127"/>
      <geometry>
        <mesh filename="../meshes/ur10e/visual/wrist3.dae"/>
      </geometry>
      <material name="ur_grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0127"/>
      <geometry>
        <mesh filename="../meshes/ur10e/collision/wrist3.stl"/>
      </geometry>
    </collision>
  </link>

  <joint name="wrist_3_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 0.1149"/>
    <parent link="wrist_3_link"/>
    <child link="tool0"/>
    <axis xyz="0 0 1"/>
    <limit effort="54" lower="-6.283185307179586" upper="6.283185307179586" velocity="3.141592653589793"/>
  </joint>

</robot>
"""


def generate_ur10e_urdf():
    out_dir = os.path.join(ROBOTS_DIR, "ur_description", "urdf")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "ur10e.urdf")
    with open(out_path, "w") as f:
        f.write(UR10E_URDF)
    print(f"  wrote {out_path}")


def main():
    print("Generating KUKA URDFs...")
    for family, stem, name in KUKA_ROBOTS:
        generate_kuka_urdf(family, stem, name)

    print("\nGenerating UR10e URDF...")
    generate_ur10e_urdf()

    print("\nDone.")


if __name__ == "__main__":
    main()

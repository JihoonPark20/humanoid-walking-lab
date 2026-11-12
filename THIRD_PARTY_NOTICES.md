# Third-party notices

Original robotics lab materials by M. Ozkan Ceylan use [Apache-2.0](LICENSE).
The attribution is preserved in [NOTICE](NOTICE).

`tools/setup_env.sh` downloads assets from
[Google DeepMind MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
at revision `370c43965323c0944b433b202aac5058b5f907a9`.
Downloaded assets retain their original licenses and notices; they are not relicensed
by this repository or bundled in the source release archive.

| Model | Installed path | License |
|---|---|---|
| Universal Robots UR5e | `third_party/mujoco_menagerie/universal_robots_ur5e/` | BSD-3-Clause |
| Robotiq 2F-85 | `third_party/mujoco_menagerie/robotiq_2f85/` | BSD-3-Clause |
| Unitree G1 | `third_party/mujoco_menagerie/unitree_g1/` | BSD-3-Clause |

The UR5e MJCF/URDF descriptions in the arm labs derive from the Menagerie model;
retain the upstream [model license](lab-2-Ur5e-robotics-lab/models/LICENSE.UR5E) when redistributing them.

[LearningHumanoidWalking](https://github.com/rohanpsingh/LearningHumanoidWalking)
(BSD-2-Clause, Rohan P. Singh; inspected revision
`cd8c6557beadb9d25045624fb50eeb487b0e8fdd`) was consulted as a research reference.
No code, models, checkpoints or media from that repository are included.

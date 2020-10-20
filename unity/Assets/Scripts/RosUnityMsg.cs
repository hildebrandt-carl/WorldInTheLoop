using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

[Serializable]
public class RosUnityMsg
{
    // Reports collisions
    public bool collision = false;

    // Gives the first drone position
    public float drone_pose_x = 0;
    public float drone_pose_y = 0;
    public float drone_pose_z = 0;
    public float drone_orientation_x = 0;
    public float drone_orientation_y = 0;
    public float drone_orientation_z = 0;

    // Gives the person position
    public float person_pose_x = 0;
    public float person_pose_y = 0;
    public float person_pose_z = 0;
    public float person_orientation_x = 0;
    public float person_orientation_y = 0;
    public float person_orientation_z = 0;

    // Returns the current image
    public byte[] image = new byte[1] {0};
}

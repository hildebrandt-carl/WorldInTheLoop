using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

[Serializable]
public class RosUnityMsg
{
    public bool collision = false;
    public float pose_x = 0;
    public float pose_y = 0;
    public float pose_z = 0;
    public float orientation_x = 0;
    public float orientation_y = 0;
    public float orientation_z = 0;
    public byte[] image = new byte[1] {0};
}

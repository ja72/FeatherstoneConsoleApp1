using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Featherstone.VectorCalculus;
using Featherstone.ScrewCalculus;
using Featherstone.Dynamics;

namespace Featherstone
{
    internal class Program
    {
        static void Main(string[] args)
        {
            TreeSystem sys = new TreeSystem();
            var L = 1.0*Vector3.UnitX;
            var Ic = ShapeMMOI.Box(1.0, 1.0, 1.0, 1.0);
            var body = new RigidBody(1.0, Ic, L/2);
            var joint = Joint.NewRevolute(Vector3.Zero, Vector3.UnitZ);

            int body_idx = sys.AddRigidBody(body);            

            int n = 6;
            for (int i = 0; i<n; i++)
            {
                sys.AddJoint(i-1, body_idx, joint);
            }
        }
    }
}

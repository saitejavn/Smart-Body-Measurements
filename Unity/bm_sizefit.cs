/*
This message class is generated automatically with 'SimpleMessageGenerator' of ROS#
*/

using Newtonsoft.Json;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.Messages.Navigation;
using RosSharp.RosBridgeClient.Messages.Sensor;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Actionlib;

namespace RosSharp.RosBridgeClient.Messages
{
    public class bm_sizefit : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "bm/bm_sizefit";

        public int size_msg;
        public int fit_msg;

        public bm_sizefit()
        {
            size_msg = -1;
            fit_msg = -1;
        }
    }
}


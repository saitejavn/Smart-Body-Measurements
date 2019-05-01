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
public class bm_fits : Message
{
[JsonIgnore]
public const string RosMessageName = "bm/bm_fits";

public int chest_fit_msg;
public int waist_fit_msg;
public int bottom_fit_msg;

public bm_fits()
{
chest_fit_msg = new int();
waist_fit_msg = new int();
bottom_fit_msg = new int();
}
}
}


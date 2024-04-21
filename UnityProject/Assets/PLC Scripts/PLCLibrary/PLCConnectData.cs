using S7.Net;

using UnityEngine;

[CreateAssetMenu(fileName = "ConnectionData", menuName = "ScriptableObjects/PLCConnectionData", order = 2)]
public class PLCConnectData : ScriptableObject
{
    public CpuType cpuType;
    public string ip;
    public short rack;
    public short slot;

    [Range(0.02f,5)]
    public float updateTime;
}
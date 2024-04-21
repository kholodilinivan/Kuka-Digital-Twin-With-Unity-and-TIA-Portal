using S7.Net;

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

using UnityEngine;

public class PLCDispatcher : MonoBehaviour
{
    public PLCConnectData conectionData;

    public List<PLCReceiveItem> receiveData = new List<PLCReceiveItem>();
    public List<PLCSendItem> sendData = new List<PLCSendItem>();

    private Dictionary<string, PLCReceiveItem> sceneReceiveDependency = new Dictionary<string, PLCReceiveItem>();
    private Plc plc;

    private void Awake()
    {
        if (conectionData == null)
        {
            Debug.LogError("[PLC] Incorrect connectionData");
            return;
        }

        plc = CreatePlc(conectionData);
        plc.Open();

        //Registrate all receive data
        var keys = new HashSet<int>();
        foreach (var receive in receiveData)
        {
            var hash = receive.data.GetHashCode();
            if (!keys.Contains(hash))
            {
                keys.Add(hash);
                var key = receive.data;
                Debug.Log($"Regitrate {key}");
                sceneReceiveDependency.Add(key, receive);
            }
        }

        //Subscribe on change sendData
        foreach (var send in sendData)
        {
            send.OnValueChange += Send;
        }

        //Start ReceiveLoop
        StartCoroutine(ReceiveLoop(conectionData.updateTime));
    }

    IEnumerator ReceiveLoop(float time)
    {
        var delay = new WaitForSeconds(time);
        var keys = sceneReceiveDependency.Keys.ToList();

        while (true)
        {
            yield return delay;
            try
            {
                foreach (var key in keys)
                {
                    var value = plc.Read(key);
                    Debug.Log($"Receive: {key} = {value}");
                    //Change values on scene
                    sceneReceiveDependency[key].ChangeState(value);
                }
            }
            catch (Exception e)
            {
                Debug.Log(e);
            }
        }
    }

    public void Send(object? value, string dataItem)
    {
        Debug.Log($"value send {dataItem} = {value}");
        plc.Write(dataItem, value);
    }

    private Plc CreatePlc(PLCConnectData data)
    {
        Debug.Log($"TryConnectTo");
        Debug.Log($"{data.cpuType}");
        Debug.Log($"{data.ip}");
        Debug.Log($"{data.rack}");
        Debug.Log($"{data.slot}");
        return new Plc(data.cpuType, data.ip, data.rack, data.slot);
    }

    public void Close()
    {
        StopAllCoroutines();
        foreach (var send in sendData)
        {
            send.OnValueChange -= Send;
        }
        plc.Close();
    }

    private void OnDestroy()
    {
        if (plc != null)
        {
            Close();
        }
    }
}

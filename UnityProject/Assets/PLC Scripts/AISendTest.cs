using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class AISendTest : MonoBehaviour
{
    public PLCSendItemInt sendItemInt;
    public System.UInt16 analogSend;
    public Slider SliderVal;

    // Update is called once per frame
    void Update()
    {
        SendTest();
    }

    public void SendTest()
    {
        analogSend = System.Convert.ToUInt16(SliderVal.value);
        sendItemInt.ChangeState(analogSend);
    }
}


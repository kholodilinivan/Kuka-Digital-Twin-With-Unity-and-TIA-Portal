using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class AIReceiveTest : MonoBehaviour
{
    public float analogReceive;
    public InputField input;

    // Update is called once per frame
    void Update()
    {
        
    }
    public void ChangeVal(System.UInt16 val)
    {
        analogReceive = val;
        input.text = val.ToString();
    }
}

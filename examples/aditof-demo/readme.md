# 3D Time of Flight : aditof-demo Sample 

#### Overview

aditof-demo example demonstrates how user can connect to ADI TOF DEPTH SENSOR device and request frames. Its written in C++ with use of OpenCV for rendering the IR and Depth image data in seperate windows.
The control window is provided to change the Range and Exposure settings of ADI TOF DEPTH SENSOR device.

#### Expected Output

When the demo is up and running, there will be three windows for IR image, Depth image and Control, as shown in below:

* Place holder for aditof-demo snapshot image

#### Code Snippets

First, initialize the ADI TOF SDK instance as below:
```
    ADI_TOF m_adiTof;
    int status = m_adiTof.Init();
    m_adiTof.setDepthRange(m_currentRange, m_currentLevel);
    m_adiTof.setPulseCount(m_currentPulse);
```

Next, get frame from the ADI TOF SDK as below:
```
    m_adiTof.ProcFrame();
    auto framePtr = std::make_shared<Frame>(m_adiTof.punDepth, m_adiTof.punIR, m_adiTof.width, m_adiTof.height);
```

Above code should be in a loop to keep getting frames as they become available with ADI TOF SDK.

Lastly, when done processing the frames, close the ADI TOF SDK instance as below:
```
m_adiTof.Release();
```



# Overview
        Bluetooth SIG provides profiles specification that defines the functionality of a Bluetooth device for different use cases, such as audio streaming, call control etc.
        All the profiles interact with Bluetooth protocol layer, application can use one or more profile libs based on their requirement and dependency.
        Below are the list of profiles supported.

##A2DP
    ###Advanced Audio Distribution Profile is a profile that allows streaming audio from one device to another. A2DP profile defines two roles.
        1. Source (SRC) – A device is the SRC when it acts as a source of a digital audio stream that is delivered to the SNK device
        2. Sink (SNK) – A device is the SNK when it acts as a sink of a digital audio stream delivered from the SRC device.

##AVRCP
    ###Audio/Video remote control profile library allows Bluetooth enabled device to control audio streaming remotely, it supports two roles
        1. Controller (CT)  –  It is a device that initiates a transaction by sending a command frame to a target.
            Examples for CT are a personal computer, a PDA, a mobile phone, a remote controller or an AV device
            (such as an in car system, headphone, player/recorder, timer, tuner, monitor etc.).
        2. Target (TG)   – Target device receives a command frame and accordingly generates a response frame. Examples for TG are an audio player/recorder,
            a video player/recorder, a TV, a tuner, an amplifier or a headphone.

##HFP
    ###Hands-free profile provides capability to the device to transmit voice between mobile phone and wireless headset and defines two roles
        1. Audio Gateway (AG) – This is the device that is the gateway of the audio, both for input and output.
            Typical devices acting as Audio Gateways are cellular phones.
        2. Hands-Free unit (HF) – This is the device acting as the Audio Gateway’s remote audio input and output
            mechanism. It also provides some remote control means to phone call functionality.
    HFP profile library CVSD, Wide band speech (WBS) as well as SWB (Super Wide Band) codec.

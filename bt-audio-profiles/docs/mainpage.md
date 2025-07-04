# Overview
        Bluetooth SIG provides profiles specification which defines functionality to Bluetooth device for different use cases, such as audio streaming, call control etc.
        All the profiles interact with Bluetooth protocol layer,application can use one or more profile libs based on their requirement and dependency.
		Below are the list of profiles supported and their version.

##A2DP
	- Advanced Audio Distribution Profile is a profile that allows streaming audio from one device to another. A2dp profile defines two roles.
		1. Source (SRC) – A device is the SRC when it acts as a source of a digital audio stream that is delivered to the SNK device
		2. Sink (SNK) – A device is the SNK when it acts as a sink of a digital audio stream delivered from the SRC device.

##AVRCP
	- Audio/Video remote control profile library allows Bluetooth enabled device to control audio streaming remotely, it supports two roles

		1. Controller (CT)  –  It is a device that initiates a transaction by sending a command frame to a target.
            Examples for CT are a personal computer, a PDA, a mobile phone, a remote controller or an AV device
            (such as an in car system, headphone, player/recorder, timer, tuner, monitor etc.).
		2. Target (TG)   – Target device receives a command frame and accordingly generates a response frame. Examples for TG are an audio player/recorder,
            a video player/recorder, a TV, a tuner, an amplifier or a headphone.

##HFP
	- Hands-free profile provides capabilty to the device to transmit voice between mobile phone and wireless headset and defines two roles
		1. Audio Gateway (AG) – This is the device that is the gateway of the audio, both for input and output.
			Typical devices acting as Audio Gateways are cellular phones.
		2. Hands-Free unit (HF) – This is the device acting as the Audio Gateway’s remote audio input and output
			mechanism. It also provides some remote control means to phone call functionality.

	HFP profile library supports both CVSD and Wide band speech(WBS) codec  features.

##SPP
	- The Serial Port Profile defines the protocols and procedures that shall be used by devices using Bluetooth for RS232 (or similar) serial cable emulation.
      it  defines two roles.
		1. Device A (DevA) – This is the device that takes initiative to form a connection to another device.
		2. Device B (DevB) – This is the device that waits for another device to take initiative to connect.
           Typical steps in using BTSTACK library APIs are -

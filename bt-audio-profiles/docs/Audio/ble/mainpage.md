# Overview
        Bluetooth SIG provides profiles specification which defines functionality to Bluetooth device for different use cases, such as audio streaming, call control etc.
        All the profiles interact with Bluetooth protocol layer,application can use one or more profile libs based on their requirement and dependency.
        Below are the list of profiles supported.

##Volume Control Service (VCS)
    - This profile enables a device to expose the controls and state of a device that can control the volume of an audio output such as one or more speakers and Control the peer device audio output as a client.
    - VCS may include zero or more instances of VOCS, AICS

##Volume Offset Control Service (VOCS)
    - Exposes the offset level and location of an audio output such as a speaker.

##Audio Input Control Service(AICS)
    - This profile exposes the settings of an audio input such as a Bluetooth audio stream, microphone, etc. Multiple audio inputs may be combined as part of the server’s audio mixing functionality.

##Microphone Input Control Service (MICS)
    - MICS is declared on devices that can control the mute state of a microphone’s audio. Only one instance of MICS is allowed.

##Audio Stream Control Service (ASCS)
    - ASCS can be instantiated on devices that can accept the establishment of unicast Audio Streams. Examples of such devices are speakers, headsets, hearing aids, earbuds, and wireless microphones.
    - Two types of ASEs :
         - Sink ASE characteristics represent Sink ASEs, to which audio data can flow. The server is said to act as Audio Sink for that ASE. There can be more than one Sink ASE characteristic on the server.
         - Source ASE characteristics represent Source ASEs, from which audio data can flow. The server is said to act as Audio Source for that ASE. There can be more than one Source ASE characteristic on the server.

##Broadcast Audio Scan Service (BASS)
    - BASS can be instantiated on servers to solicit for clients to scan on behalf of the server for broadcast Audio Streams and associated data that are transmitted by Broadcast Sources.
    - Clients scanning on behalf of the server can help reduce the need to scan by the server and reduce power consumption on the server.
    - Servers can receive information from clients that is associated with broadcast Audio Streams, including decryption keys known as Broadcast_Codes necessary to decrypt encrypted BISes.

##Coordinated Set Identification Service (CSIS)
    - The Coordinated Set Identification Service (CSIS) can be used by devices to be discovered as part of a Coordinated Set. A Coordinated Set is defined as a group of devices that are configured to support a specific scenario.
    - Examples of Coordinated Sets include a pair of hearing aids, a pair of earbuds, or a speaker set that receives multi-channel audio and that reacts to control commands in a coordinated way (e.g., volume up and volume down).
    - CSIS is agnostic to the actual features and functions of the devices. The purpose of CSIS is to specify how a device can be discovered as part of a Coordinated Set and how to grant a client exclusive access to the Coordinated Set to avoid race conditions when multiple clients want to access the Coordinated Set at the same time.

##Media Control Service (MCS)/(GMCS)
    - MCS/GMCS exposes characteristics that describe a single media player. A media player in this context is a device, or part of a device, that allows another device to control the media that is played. For example, a media player could be a television, a set-top box, a radio, a phone running a radio or music application, a
      phone running a podcast application, or a similar type of device or application. MCS does not directly manage the audio transport
    - GMCS provides status and control of media playback for the device as a single unit.

##Telephone Bearer Service (TBS)/(GTBS)
    - TBS and GTBS both expose characteristics that provide status and control of the telephone bearers. A device can have multiple, logically separate, telephone bearers. A TBS instance provides status and control for a specific telephone bearer within the device. A device implements TBS instances when the
     device wants to allow clients to directly access and control the characteristics of separate internal telephone bearer entities.
    - GTBS provides a single point of access and exposes a representation of its internal telephone bearers into a single telephone bearer.

##Published Audio Capabilities (PACS)
    - PACS can be instantiated on devices that can accept the establishment of unicast Audio Streams or devices that can receive broadcast Audio Streams. Examples of such devices are speakers, headsets, hearing aids, and microphones.
    - Servers expose one or more sets of audio capabilities and audio availability. Sets of audio capabilities,known as Published Audio Capability (PAC) records, are exposed by using either the Sink PAC characteristic or Source PAC characteristic. Clients can discover and read these characteristics, and servers can notify these characteristics.

##Public Broadcast profile (PBP)
    - Public Broadcast Profile (PBP) defines a Public Broadcast Announcement that a Broadcast Source can include in an extended advertisement to indicate that a Broadcast Source is transmitting at least one of the following:
        - A broadcast Audio Stream with a configuration that every BAP Broadcast Sink can receive and decode (Standard Quality Public Broadcast Audio)
        - A broadcast Audio Stream that uses a High Quality Public Broadcast Audio Stream configuration
    - The Public Broadcast Announcement also indicates whether the broadcast Audio Streams in the Broadcast Isochronous Group (BIG) are encrypted or not. If the Public Broadcast Announcement indicates that the broadcast Audio Streams are encrypted, a Broadcast Sink must use a Broadcast_Code to decrypt them

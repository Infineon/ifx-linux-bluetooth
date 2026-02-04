# Bluetooth Classic Profiles

## Changelog
=============
## v2.0.1
- Warning fixes.
- Fixed docs URL on github.
- HFP fix for corner case of both sides trying to setup connection simultaneously.

## v2.0.0
- Updated A2DP to support 1.4
    - Added support for MPEG-D USAC and updates for DRC in MPEG-2,4.
    - Added wiced_bt_a2dp_source_init_ext to effectively support multiple codecs for source.
- Updated HFP to support 1.9
    - Added support for 3-way-calling and SWB for HF role.
    - Added support for 3-way-calling, ECNR, In-band ringtone, SWB, HF indicators and ECC for AG role.
    - Breaking Change: wbs_supported and wbs_used removed from wiced_bt_hfp_ag_audio_open_t.
- Updated AVRCP to support 1.6.3
- Warning fixes in A2DP Source and AVRCP

## v1.0.1
- Memory leak issue fixed in avrc_controller profile
- CoverArt feature is not supported in AVRCP profile

## v1.0.0
- Initial public release of Bluetooth Classic Profiles

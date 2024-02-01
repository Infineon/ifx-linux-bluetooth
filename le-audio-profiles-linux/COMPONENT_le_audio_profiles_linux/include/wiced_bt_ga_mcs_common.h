/*
* $ Copyright Cypress Semiconductor $
*/

#ifndef __WICED_BT_GA_MCS_COMMON_H__
#define __WICED_BT_GA_MCS_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

// Maximum Media Player length supported by application
#define MAX_MEDIA_PLAYER_NAME_LEN      50
// Maximum Track Title length supported by application
#define MAX_MEDIA_TRACK_TITLE_LEN            100
// Maximum Event Data Length
#define MAX_MEDIA_EVENT_DATA_LENGTH          100
// Minimum playback speed supported by MCS
#define MIN_MEDIA_PLAYBACK_SPEED    (-128)
// Maximum playback speed supported by MCS
#define MAX_MEDIA_PLAYBACK_SPEED    (127)

/* Media Playing order (Media Control Service Section 3.15 Playing Order)*/
typedef enum
{
    WICED_BT_GA_MCS_SINGLE_ONCE     = 0x01,        /**< A single track is played once; there is no next track */
    WICED_BT_GA_MCS_SINGLE_REPEAT   = 0x02,        /**< A single track is played repeatedly; the next track is the current track */
    WICED_BT_GA_MCS_IN_ORDER_ONCE   = 0x03,        /**< The tracks within a group are played once in track order */
    WICED_BT_GA_MCS_IN_ORDER_REPEAT = 0x04,        /**< The tracks within a group are played in track order repeatedly */
    WICED_BT_GA_MCS_OLDEST_ONCE     = 0x05,        /**< The tracks within a group are played once only from the oldest first */
    WICED_BT_GA_MCS_OLDEST_REPEAT   = 0x06,        /**< The tracks within a group are played from the oldest first repeatedly. */
    WICED_BT_GA_MCS_NEWEST_ONCE     = 0x07,        /**< The tracks within a group are played once only from the newest first */
    WICED_BT_GA_MCS_NEWEST_REPEAT   = 0x08,        /**< The tracks within a group are played from the newest first repeatedly */
    WICED_BT_GA_MCS_SHUFFLE_ONCE    = 0x09,        /**< The tracks within a group are played in random order once */
    WICED_BT_GA_MCS_SHUFFLE_REPEAT  = 0x0A,        /**< The tracks within a group are played in random order repeatedly */
} wiced_bt_ga_media_control_playing_order_t;

/* Media Playing order Supported (Media Control Service Section 3.16 Playing Order Supported)*/
#define MCS_SINGLE_ONCE_PLAYING_ORDER_MASK          (1<<0)      /**< A single track is played once; there is no next track */
#define MCS_SINGLE_REPEAT_PLAYING_ORDER_MASK        (1<<1)      /**< A single track is played repeatedly; the next track is the current track */
#define MCS_IN_ORDER_ONCE_PLAYING_ORDER_MASK        (1<<2)      /**< The tracks within a group are played once in track order */
#define MCS_IN_ORDER_REPEAT_PLAYING_ORDER_MASK      (1<<3)      /**< The tracks within a group are played in track order repeatedly */
#define MCS_OLDEST_ONCE_PLAYING_ORDER_MASK          (1<<4)      /**< The tracks within a group are played once only from the oldest first */
#define MCS_OLDEST_REPEAT_PLAYING_ORDER_MASK        (1<<5)      /**< The tracks within a group are played from the oldest first repeatedly */
#define MCS_NEWEST_ONCE_PLAYING_ORDER_MASK          (1<<6)      /**< The tracks within a group are played once only from the newest first */
#define MCS_NEWEST_REPEAT_PLAYING_ORDER_MASK        (1<<7)      /**< The tracks within a group are played from the newest first repeatedly */
#define MCS_SHUFFLE_ONCE_PLAYING_ORDER_MASK         (1<<8)      /**< The tracks within a group are played in random order once */
#define MCS_SHUFFLE_REPEAT_PLAYING_ORDER_MASK       (1<<9)      /**< The tracks within a group are played in random order repeatedly */


/* Media State (Media Control Service Section 3.17 Media State)*/
typedef enum
{
    WICED_BT_GA_MCS_MEDIA_INACTIVE  = 0x00,        /**< The current track is inactive */
    WICED_BT_GA_MCS_MEDIA_PLAYING   = 0x01,        /**< The current track is playing */
    WICED_BT_GA_MCS_MEDIA_PAUSED    = 0x02,        /**< The current track is paused */
    WICED_BT_GA_MCS_MEDIA_SEEKING   = 0x03,        /**< The current track is fast forwarding or fast rewinding */
} wiced_bt_ga_media_control_state_t;

/* Media Control Opcode (Media Control Service Section 3.18 Media Control Point)*/
typedef enum
{
    WICED_BT_GA_MCS_INVALID           = 0x00,      /**< Invalid Operation ID */
    WICED_BT_GA_MCS_PLAY              = 0x01,      /**< Start playing the current track */
    WICED_BT_GA_MCS_PAUSE             = 0x02,      /**< Pause playing the current track */
    WICED_BT_GA_MCS_FAST_REWIND       = 0x03,      /**< Fast rewind the current track */
    WICED_BT_GA_MCS_FAST_FORWARD      = 0x04,      /**< Fast forward the current track */
    WICED_BT_GA_MCS_STOP              = 0x05,      /**< Stop current activity and return to stopped state */
    WICED_BT_GA_MCS_MOVE_RELATIVE     = 0x10,      /**< Set the current position relative to the current position */
    WICED_BT_GA_MCS_PREVIOUS_SEGMENT  = 0x20,      /**< Set the current position to the previous segment of the current track */
    WICED_BT_GA_MCS_NEXT_SEGMENT      = 0x21,      /**< Set the current position to the next segment of the current track */
    WICED_BT_GA_MCS_FIRST_SEGMENT     = 0x22,      /**< Set the current position to the first segment of the current track */
    WICED_BT_GA_MCS_LAST_SEGMENT      = 0x23,      /**< Set the current position to the last segment of the current track */
    WICED_BT_GA_MCS_GOTO_SEGMENT      = 0x24,      /**< Set the current position to the nth segment of the current track */
    WICED_BT_GA_MCS_PREVIOUS_TRACK    = 0x30,      /**< Set the current track to the previous track in the current group playing order */
    WICED_BT_GA_MCS_NEXT_TRACK        = 0x31,      /**< Set the current track to the next track in the current group playing order */
    WICED_BT_GA_MCS_FIRST_TRACK       = 0x32,      /**< Set the current track to the first track in the current group playing order */
    WICED_BT_GA_MCS_LAST_TRACK        = 0x33,      /**< Set the current track to the last track in the current group playing order */
    WICED_BT_GA_MCS_GOTO_TRACK        = 0x34,      /**< Set the current track to the nth track in the current group playing order */
    WICED_BT_GA_MCS_PREVIOUS_GROUP    = 0x40,      /**< Set the current group to the previous group in the sequence of groups */
    WICED_BT_GA_MCS_NEXT_GROUP        = 0x41,      /**< Set the current group to the next group in the sequence of groups */
    WICED_BT_GA_MCS_FIRST_GROUP       = 0x42,      /**< Set the current group to the first group in the sequence of groups */
    WICED_BT_GA_MCS_LAST_GROUP        = 0x43,      /**< Set the current group to the last group in the sequence of groups */
    WICED_BT_GA_MCS_GOTO_GROUP        = 0x44,      /**< Set the current group to the nth group in the sequence of groups */
} wiced_bt_ga_mcp_media_control_operation_t;

/* Media Control Opcode (Media Control Service Section 3.18 Media Control Point)*/
typedef enum
{
    WICED_BT_GA_MCS_SUCCESS                        = 1,         /**< Action requested by the opcode write was completed successfully */
    WICED_BT_GA_MCS_OPCODE_NOT_SUPPORTED           = 2,         /**< An invalid opcode was used for the Media Control Point write. */
    WICED_BT_GA_MCS_MEDIA_PLAYER_INACTIVE          = 3,         /**< The Media Player State characteristic value is Inactive */
    WICED_BT_GA_MCS_COMMAND_CANNOT_BE_COMPLETED    = 4,         /**< The requested action of any Media Control Point write cannot be completed successfully due to a condition within the player */
}wiced_bt_ga_mcp_result_t;

/* Media Control Opcode Supported (Media Control Service Section 3.19 Media Control Point Opcodes Supported)*/
#define MCS_PLAY_OPCODE_SUPPORTED_MASK                 (1<<0)       /**< Start playing the current track */
#define MCS_PAUSE_OPCODE_SUPPORTED_MASK                (1<<1)       /**< Pause playing the current track */
#define MCS_FAST_REWIND_OPCODE_SUPPORTED_MASK          (1<<2)       /**< Fast forward the current track */
#define MCS_FAST_FORWARD_OPCODE_SUPPORTED_MASK         (1<<3)       /**< Fast rewind the current track */
#define MCS_STOP_OPCODE_SUPPORTED_MASK                 (1<<4)       /**< Stop current activity and return to stopped state */
#define MCS_MOVE_RELATIVE_OPCODE_SUPPORTED_MASK        (1<<5)       /**< Set the current position relative to the current position */
#define MCS_PREVIOUS_SEGMENT_OPCODE_SUPPORTED_MASK     (1<<6)       /**< Set the current position to the previous segment of the current track */
#define MCS_NEXT_SEGMENT_OPCODE_SUPPORTED_MASK         (1<<7)       /**< Set the current position to the next segment of the current track */
#define MCS_FIRST_SEGMENT_OPCODE_SUPPORTED_MASK        (1<<8)       /**< Set the current position to the first segment of the current track */
#define MCS_LAST_SEGMENT_OPCODE_SUPPORTED_MASK         (1<<9)       /**< Set the current position to the last segment of the current track */
#define MCS_GOTO_SEGMENT_OPCODE_SUPPORTED_MASK         (1<<10)      /**< Set the current position to the nth segment of the current track */
#define MCS_PREVIOUS_TRACK_OPCODE_SUPPORTED_MASK       (1<<11)      /**< Set the current track to the previous track in the current group playing order */
#define MCS_NEXT_TRACK_OPCODE_SUPPORTED_MASK           (1<<12)      /**< Set the current track to the next track in the current group playing order */
#define MCS_FIRST_TRACK_OPCODE_SUPPORTED_MASK          (1<<13)      /**< Set the current track to the first track in the current group playing order */
#define MCS_LAST_TRACK_OPCODE_SUPPORTED_MASK           (1<<14)      /**< Set the current track to the last track in the current group playing order */
#define MCS_GOTO_TRACK_OPCODE_SUPPORTED_MASK           (1<<15)      /**< Set the current track to the nth track in the current group playing order */
#define MCS_PREVIOUS_GROUP_OPCODE_SUPPORTED_MASK       (1<<16)      /**< Set the current group to the previous group in the sequence of groups */
#define MCS_NEXT_GROUP_OPCODE_SUPPORTED_MASK           (1<<17)      /**< Set the current group to the next group in the sequence of groups */
#define MCS_FIRST_GROUP_OPCODE_SUPPORTED_MASK          (1<<18)      /**< Set the current group to the first group in the sequence of groups */
#define MCS_LAST_GROUP_OPCODE_SUPPORTED_MASK           (1<<19)      /**< Set the current group to the last group in the sequence of groups */
#define MCS_GOTO_GROUP_OPCODE_SUPPORTED_MASK           (1<<20)      /**< Set the current group to the nth group in the sequence of groups */

#define MCS_BASIC_OPCODES_SUPPORTED       ( MCS_PLAY_OPCODE_SUPPORTED_MASK | \
                                        MCS_PAUSE_OPCODE_SUPPORTED_MASK | \
                                        MCS_FAST_REWIND_OPCODE_SUPPORTED_MASK  | \
                                        MCS_FAST_FORWARD_OPCODE_SUPPORTED_MASK | \
                                        MCS_STOP_OPCODE_SUPPORTED_MASK | \
                                        MCS_MOVE_RELATIVE_OPCODE_SUPPORTED_MASK )

#define MCS_SEGMENT_OPCODES_SUPPORTED     ( MCS_PREVIOUS_SEGMENT_OPCODE_SUPPORTED_MASK | \
                                        MCS_NEXT_SEGMENT_OPCODE_SUPPORTED_MASK | \
                                        MCS_FIRST_SEGMENT_OPCODE_SUPPORTED_MASK  | \
                                        MCS_LAST_SEGMENT_OPCODE_SUPPORTED_MASK | \
                                        MCS_GOTO_SEGMENT_OPCODE_SUPPORTED_MASK )

#define MCS_TRACK_OPCODES_SUPPORTED       ( MCS_PREVIOUS_TRACK_OPCODE_SUPPORTED_MASK | \
                                        MCS_NEXT_TRACK_OPCODE_SUPPORTED_MASK | \
                                        MCS_FIRST_TRACK_OPCODE_SUPPORTED_MASK  | \
                                        MCS_LAST_TRACK_OPCODE_SUPPORTED_MASK | \
                                        MCS_GOTO_TRACK_OPCODE_SUPPORTED_MASK )

#define MCS_GROUP_OPCODES_SUPPORTED       ( MCS_PREVIOUS_GROUP_OPCODE_SUPPORTED_MASK | \
                                        MCS_NEXT_GROUP_OPCODE_SUPPORTED_MASK | \
                                        MCS_FIRST_GROUP_OPCODE_SUPPORTED_MASK  | \
                                        MCS_LAST_GROUP_OPCODE_SUPPORTED_MASK | \
                                        MCS_GOTO_GROUP_OPCODE_SUPPORTED_MASK )

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_GA_MCS_COMMON_H__ */

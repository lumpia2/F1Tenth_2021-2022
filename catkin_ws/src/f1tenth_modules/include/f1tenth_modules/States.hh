#ifndef STATES_H_
#define STATES_H_

#include <string>

/**
 * @brief Mux states that control the drive topic
 *
 */
typedef enum MuxStates
{
    OFF=0,
    MANUAL,
    WALLFOLLOWING,
    GAPFOLLOWING,

    _SIZE_
};


namespace States {

    typedef struct Manual
    {
        const std::string driveTopic = "/manual";
    };

    typedef struct WallFollowing
    {
        const std::string driveTopic = "/wall_following";
    };

    typedef struct GapFollowing
    {
        const std::string driveTopic = "/gap_following";
    };

} // namespace States


#endif
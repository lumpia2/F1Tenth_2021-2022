#ifndef MUX_H_
#define MUX_H_

#include <ros/ros.h>

class Mux
{
private:
    std::vector<std::string> registeredInputs;

    // callback for the switching
public:
//register topic(input)
    void registerTopic(const std::string &);
//publish to main drive topic(ouput)
    void publishDrive()
};


#endif

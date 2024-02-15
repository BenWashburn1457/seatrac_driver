#ifndef _DEF_SEATRAC_DRIVER_PRINT_UTILS_H_
#define _DEF_SEATRAC_DRIVER_PRINT_UTILS_H_

#include <iostream>

namespace narval { namespace seatrac { namespace print_utils {

template <typename T>
std::string indent(const T& input, const char* indent = "  ")
{
    std::ostringstream tmp;
    tmp << input;

    std::ostringstream oss;
    for(auto c : tmp.str()) {
        oss << c;
        if(c == '\n')
            oss << indent;
    }
    return oss.str();
}

template <typename T>
std::string to_string(const T& input)
{
    std::ostringstream oss;
    oss << input;
    return oss.str();
}

}; //namespace print_utils
}; //namespace seatrac
}; //namespace narval

#endif //_DEF_SEATRAC_DRIVER_PRINT_UTILS_H_

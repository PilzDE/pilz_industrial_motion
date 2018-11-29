#ifndef EXCEPTION_TYPES_H
#define EXCEPTION_TYPES_H

#include <stdexcept>
#include <string>

namespace pilz_industrial_motion_testutils
{

class TestDataLoaderReadingException: public std::runtime_error
{
  public:
    TestDataLoaderReadingException(const std::string error_desc)
      : std::runtime_error(error_desc)
    {}
};

}

#endif // EXCEPTION_TYPES_H

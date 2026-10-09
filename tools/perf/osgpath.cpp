#include <deque>
#include <string>

namespace osgDB {

// Headless measurements need no plugin discovery or conda-embedded prefix.
void appendPlatformSpecificLibraryFilePaths(std::deque<std::string>&)
{
}

} // namespace osgDB

#ifndef SRC_UTILS_FILE_MANIPULATION_H
#define SRC_UTILS_FILE_MANIPULATION_H

#include <string>
using namespace std;

namespace utils {
  namespace filemanip {
    bool check_file_exists(const char* const filename);
    bool create_backup(const char* const filename);
    bool create_backup(const char* const source,
                       const char* const dest);

    string parent_path(const char* const filename);
    string stem(const char* const filename);
    string extension(const char* const filename);
    string stem_ext(const char* const filename);
  } // namespace filemanip
} // namespace utils

#endif // #ifndef SRC_UTILS_FILE_MANIPULATION_H

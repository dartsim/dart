#include "file_manipulation.h"

#include <string>
#include <iostream>
using namespace std;
// Boost Libraries
#include "boost/scoped_ptr.hpp"
#include "boost/filesystem.hpp"
namespace bf = boost::filesystem;

namespace utils {
  namespace filemanip {

    bool check_file_exists(const char* const filename) {
      return bf::exists(filename);
    }

    bool create_backup(const char* const filename) {
      if (!bf::exists(filename)) {
        return false;
      }
      // Recombine the file name
      bf::path p(filename);

      string parent_path = p.parent_path().string();
      string file = p.filename();
      string bk_filename = parent_path + "/bk." + file;
      
      // copy_file(from, to)
      if (!bf::exists(bk_filename)) {
        bf::copy_file(filename, bk_filename);
      }
      return true;
    }

    bool create_backup(const char* const source,
                       const char* const dest) {
      // copy_file(from, to)
      bf::copy_file(source, dest);
      return true;
    }

    string parent_path(const char* const filename) {
      bf::path p(filename);
      return p.parent_path().string();
    }

    string stem(const char* const filename) {
      bf::path p(filename);
      return p.stem();
      
    }

    string extension(const char* const filename) {
      bf::path p(filename);
      return p.extension();
    }

    string stem_ext(const char* const filename) {
      bf::path p(filename);
      return p.filename();
    }
    
  } // namespace filemanip
} // namespace utils

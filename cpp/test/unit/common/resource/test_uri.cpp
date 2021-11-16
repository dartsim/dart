/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include "dart/common/resource/uri.hpp"

using namespace dart;
using namespace common;

//==============================================================================
TEST(UriTest, from_string_ValidUri_ReturnsTrue)
{
  // These examples are from Section 1.1.2 of RFC 3986.
  Uri uri;

  ASSERT_TRUE(uri.from_string("ftp://ftp.is.co.za/rfc/rfc1808.txt"));
  ASSERT_TRUE(uri.scheme);
  ASSERT_TRUE(uri.authority);
  ASSERT_TRUE(uri.path);
  EXPECT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("ftp", *uri.scheme);
  EXPECT_EQ("ftp.is.co.za", *uri.authority);
  EXPECT_EQ("/rfc/rfc1808.txt", *uri.path);

  ASSERT_TRUE(uri.from_string("http://www.ietf.org/rfc/rfc2396.txt"));
  ASSERT_TRUE(uri.scheme);
  ASSERT_TRUE(uri.authority);
  ASSERT_TRUE(uri.path);
  EXPECT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("http", *uri.scheme);
  EXPECT_EQ("www.ietf.org", *uri.authority);
  EXPECT_EQ("/rfc/rfc2396.txt", *uri.path);

  ASSERT_TRUE(uri.from_string("ldap://[2001:db8::7]/c=GB?objectClass?one"));
  ASSERT_TRUE(uri.scheme);
  ASSERT_TRUE(uri.authority);
  ASSERT_TRUE(uri.path);
  ASSERT_TRUE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("ldap", *uri.scheme);
  EXPECT_EQ("[2001:db8::7]", *uri.authority);
  EXPECT_EQ("/c=GB", *uri.path);
  EXPECT_EQ("objectClass?one", *uri.query);

  ASSERT_TRUE(uri.from_string("mailto:John.Doe@example.com"));
  ASSERT_TRUE(uri.scheme);
  ASSERT_FALSE(uri.authority);
  ASSERT_TRUE(uri.path);
  ASSERT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("mailto", *uri.scheme);
  EXPECT_EQ("John.Doe@example.com", *uri.path);

  ASSERT_TRUE(uri.from_string("news:comp.infosystems.www.servers.unix"));
  ASSERT_TRUE(uri.scheme);
  ASSERT_FALSE(uri.authority);
  ASSERT_TRUE(uri.path);
  ASSERT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("news", *uri.scheme);
  EXPECT_EQ("comp.infosystems.www.servers.unix", *uri.path);

  ASSERT_TRUE(uri.from_string("tel:+1-816-555-1212"));
  ASSERT_TRUE(uri.scheme);
  ASSERT_FALSE(uri.authority);
  ASSERT_TRUE(uri.path);
  ASSERT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("tel", *uri.scheme);
  EXPECT_EQ("+1-816-555-1212", *uri.path);

  ASSERT_TRUE(uri.from_string("telnet://192.0.2.16:80/"));
  ASSERT_TRUE(uri.scheme);
  ASSERT_TRUE(uri.authority);
  ASSERT_TRUE(uri.path);
  ASSERT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("telnet", *uri.scheme);
  EXPECT_EQ("192.0.2.16:80", *uri.authority);
  EXPECT_EQ("/", *uri.path);

  ASSERT_TRUE(
      uri.from_string("urn:oasis:names:specification:docbook:dtd:xml:4.1.2"));
  ASSERT_TRUE(uri.scheme);
  ASSERT_FALSE(uri.authority);
  ASSERT_TRUE(uri.path);
  ASSERT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_FALSE(uri.authority);
  EXPECT_EQ("oasis:names:specification:docbook:dtd:xml:4.1.2", *uri.path);
}

TEST(UriTest, from_path_PathNotUri_ReturnsFileURIwithEmptyAuthority)
{
#ifdef _WIN32
  std::vector<std::string> testPaths = {
      "C:\\foo",
      "C:\\foo\\",
      "C:\\foo\\bar",
  };
#else
  std::vector<std::string> testPaths = {"/foo", "/foo/", "/foo/bar"};
#endif

  Uri uri;

  ASSERT_TRUE(uri.from_path(testPaths[0]));
  ASSERT_TRUE(uri.scheme);
  ASSERT_TRUE(uri.authority);
  ASSERT_TRUE(uri.path);
  ASSERT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("file", *uri.scheme);
  EXPECT_EQ("", *uri.authority);
  EXPECT_EQ(testPaths[0], uri.get_filesystem_path());

  ASSERT_TRUE(uri.from_path(testPaths[1]));
  ASSERT_TRUE(uri.scheme);
  ASSERT_TRUE(uri.authority);
  ASSERT_TRUE(uri.path);
  ASSERT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("file", *uri.scheme);
  EXPECT_EQ("", *uri.authority);
  EXPECT_EQ(testPaths[1], uri.get_filesystem_path());

  ASSERT_TRUE(uri.from_path(testPaths[2]));
  ASSERT_TRUE(uri.scheme);
  ASSERT_TRUE(uri.authority);
  ASSERT_TRUE(uri.path);
  ASSERT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("file", *uri.scheme);
  EXPECT_EQ("", *uri.authority);
  EXPECT_EQ(testPaths[2], uri.get_filesystem_path());
}

TEST(UriTest, from_string_or_path_UriNotPathNorFileUri_ReturnsUriNotFileUri)
{
  std::vector<std::string> testUris
      = {"ftp://ftp.is.co.za/rfc/rfc1808.txt",
         "http://www.ietf.org/rfc/rfc2396.txt",
         "ldap://[2001:db8::7]/c=GB?objectClass?one",
         "mailto:John.Doe@example.com",
         "news:comp.infosystems.www.servers.unix",
         "tel:+1-816-555-1212",
         "telnet://192.0.2.16:80/",
         "urn:oasis:names:specification:docbook:dtd:xml:4.1.2"};

  Uri uri;

  for (const std::string& testUri : testUris) {
    uri.from_string_or_path(testUri);

    EXPECT_NE("file", *uri.scheme);
    EXPECT_EQ(testUri, uri.to_string());
  }
}

TEST(UriTest, from_string_InputIsUri_DoesNotChange)
{
  Uri uri;

  ASSERT_TRUE(uri.from_string("ftp://ftp.is.co.za/rfc/rfc1808.txt"));
  ASSERT_TRUE(uri.scheme);
  ASSERT_TRUE(uri.authority);
  ASSERT_TRUE(uri.path);
  EXPECT_FALSE(uri.query);
  EXPECT_FALSE(uri.fragment);
  EXPECT_EQ("ftp", *uri.scheme);
  EXPECT_EQ("ftp.is.co.za", *uri.authority);
  EXPECT_EQ("/rfc/rfc1808.txt", *uri.path);
}

TEST(UriTest, GetUri_InputIsUri_DoesNotChange)
{
  std::vector<std::string> testUris
      = {"ftp://ftp.is.co.za/rfc/rfc1808.txt",
         "http://www.ietf.org/rfc/rfc2396.txt",
         "ldap://[2001:db8::7]/c=GB?objectClass?one",
         "mailto:John.Doe@example.com",
         "news:comp.infosystems.www.servers.unix",
         "tel:+1-816-555-1212",
         "telnet://192.0.2.16:80/",
         "urn:oasis:names:specification:docbook:dtd:xml:4.1.2"};

  for (const std::string& testUri : testUris)
    EXPECT_EQ(testUri, Uri::CreateFromString(testUri).to_string());
}

TEST(UriTest, GetUri_InputIsPath_AppendsFileSchema)
{
#ifdef _WIN32
  std::vector<std::string> testPaths = {
      "C:\\foo",
      "C:\\foo\\",
      "C:\\foo\\bar",
  };
#else
  std::vector<std::string> testPaths = {"/foo", "/foo/", "/foo/bar"};
#endif

  for (const std::string& testPath : testPaths) {
#ifdef _WIN32
    // On Windows, an absolute path does not begin with forward slash but a
    // file URI needs it to represent an empty authority,
    const std::string testUri = "file:///" + testPath;
#else
    // wherease on Unix systems the additional forward slash is not required
    // since an absolute path already has it.
    const std::string testUri = "file://" + testPath;
#endif
    EXPECT_EQ(testUri, Uri::GetUri(testPath));
  }
}

TEST(UriTest, getRelativeUri)
{
  std::vector<std::pair<std::string, std::string> > testPairs = {
      // RFC 3986, Section 5.4.1.: Normal Examples
      {"g:h", "g:h"},
      {"g", "http://a/b/c/g"},
      {"./g", "http://a/b/c/g"},
      {"g/", "http://a/b/c/g/"},
      {"/g", "http://a/g"},
      {"//g", "http://g"},
      {"?y", "http://a/b/c/d;p?y"},
      {"g?y", "http://a/b/c/g?y"},
      {"#s", "http://a/b/c/d;p?q#s"},
      {"g#s", "http://a/b/c/g#s"},
      {"g?y#s", "http://a/b/c/g?y#s"},
      {";x", "http://a/b/c/;x"},
      {"g;x", "http://a/b/c/g;x"},
      {"g;x?y#s", "http://a/b/c/g;x?y#s"},
      {"", "http://a/b/c/d;p?q"},
      {".", "http://a/b/c/"},
      {"./", "http://a/b/c/"},
      {"..", "http://a/b/"},
      {"../", "http://a/b/"},
      {"../g", "http://a/b/g"},
      {"../..", "http://a/"},
      {"../../", "http://a/"},
      {"../../g", "http://a/g"},
      // RFC 3986, Section 5.4.2.: Abnormal Examples
      {"../../../g", "http://a/g"},
      {"../../../../g", "http://a/g"},
      {"/./g", "http://a/g"},
      {"/../g", "http://a/g"},
      {"g.", "http://a/b/c/g."},
      {".g", "http://a/b/c/.g"},
      {"g..", "http://a/b/c/g.."},
      {"..g", "http://a/b/c/..g"},
      {"./../g", "http://a/b/g"},
      {"./g/.", "http://a/b/c/g/"},
      {"g/./h", "http://a/b/c/g/h"},
      {"g/../h", "http://a/b/c/h"},
      {"g;x=1/./y", "http://a/b/c/g;x=1/y"},
      {"g;x=1/../y", "http://a/b/c/y"},
      {"g?y/./x", "http://a/b/c/g?y/./x"},
      {"g?y/../x", "http://a/b/c/g?y/../x"},
      {"g#s/./x", "http://a/b/c/g#s/./x"},
      {"g#s/../x", "http://a/b/c/g#s/../x"},
  };

  Uri base_uri, relative_uri, mergedUri;
  ASSERT_TRUE(base_uri.from_string("http://a/b/c/d;p?q"));

  for (const auto& it : testPairs) {
    const std::string& expectedUri = it.second;

    ASSERT_TRUE(relative_uri.from_string(it.first));

    // Strict mode
    ASSERT_TRUE(mergedUri.from_relative_uri(base_uri, relative_uri, true));
    EXPECT_EQ(expectedUri, mergedUri.to_string());

    // Backwards compatability mode
    ASSERT_TRUE(mergedUri.from_relative_uri(base_uri, relative_uri, false));
    EXPECT_EQ(expectedUri, mergedUri.to_string());
  }

  // Strict mode behavior.
  ASSERT_TRUE(relative_uri.from_string("http:g"));
  ASSERT_TRUE(mergedUri.from_relative_uri(base_uri, "http:g", true));
  EXPECT_EQ("http:g", mergedUri.to_string());

  // TODO: I'm not sure what we have to do to implement this. The approach
  // described in the RFC doesn't seem to produce the expected result.
#if 0
  // Backwards compatability mode behavior.
  ASSERT_TRUE(relative_uri.from_string("http:g"));
  ASSERT_TRUE(mergedUri.from_relative_uri(base_uri, "http:g", false));
  EXPECT_EQ("http://a/b/c/g", mergedUri.to_string());
#endif
}

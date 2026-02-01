/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "../../helpers/gtest_utils.hpp"
#include "dart/common/uri.hpp"

#include <gtest/gtest.h>

#include <string_view>

using dart::common::Uri;
using dart::common::UriComponent;

TEST(UriHelpers, fromString_ValidUri_ReturnsTrue)
{
  // These examples are from Section 1.1.2 of RFC 3986.
  Uri uri;

  ASSERT_TRUE(uri.fromString("ftp://ftp.is.co.za/rfc/rfc1808.txt"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  EXPECT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("ftp", *uri.mScheme);
  EXPECT_EQ("ftp.is.co.za", *uri.mAuthority);
  EXPECT_EQ("/rfc/rfc1808.txt", *uri.mPath);

  ASSERT_TRUE(uri.fromString("http://www.ietf.org/rfc/rfc2396.txt"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  EXPECT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("http", *uri.mScheme);
  EXPECT_EQ("www.ietf.org", *uri.mAuthority);
  EXPECT_EQ("/rfc/rfc2396.txt", *uri.mPath);

  ASSERT_TRUE(uri.fromString("ldap://[2001:db8::7]/c=GB?objectClass?one"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_TRUE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("ldap", *uri.mScheme);
  EXPECT_EQ("[2001:db8::7]", *uri.mAuthority);
  EXPECT_EQ("/c=GB", *uri.mPath);
  EXPECT_EQ("objectClass?one", *uri.mQuery);

  ASSERT_TRUE(uri.fromString("mailto:John.Doe@example.com"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("mailto", *uri.mScheme);
  EXPECT_EQ("John.Doe@example.com", *uri.mPath);

  ASSERT_TRUE(uri.fromString("news:comp.infosystems.www.servers.unix"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("news", *uri.mScheme);
  EXPECT_EQ("comp.infosystems.www.servers.unix", *uri.mPath);

  ASSERT_TRUE(uri.fromString("tel:+1-816-555-1212"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("tel", *uri.mScheme);
  EXPECT_EQ("+1-816-555-1212", *uri.mPath);

  ASSERT_TRUE(uri.fromString("telnet://192.0.2.16:80/"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("telnet", *uri.mScheme);
  EXPECT_EQ("192.0.2.16:80", *uri.mAuthority);
  EXPECT_EQ("/", *uri.mPath);

  ASSERT_TRUE(
      uri.fromString("urn:oasis:names:specification:docbook:dtd:xml:4.1.2"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_FALSE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_FALSE(uri.mAuthority);
  EXPECT_EQ("oasis:names:specification:docbook:dtd:xml:4.1.2", *uri.mPath);
}

TEST(UriHelpers, fromPath_PathNotUri_ReturnsFileURIwithEmptyAuthority)
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

  ASSERT_TRUE(uri.fromPath(testPaths[0]));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("", *uri.mAuthority);
  EXPECT_EQ(testPaths[0], uri.getFilesystemPath());

  ASSERT_TRUE(uri.fromPath(testPaths[1]));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("", *uri.mAuthority);
  EXPECT_EQ(testPaths[1], uri.getFilesystemPath());

  ASSERT_TRUE(uri.fromPath(testPaths[2]));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  ASSERT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("", *uri.mAuthority);
  EXPECT_EQ(testPaths[2], uri.getFilesystemPath());
}

TEST(UriHelpers, fromStringOrPath_UriNotPathNorFileUri_ReturnsUriNotFileUri)
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
    uri.fromStringOrPath(testUri);

    EXPECT_NE("file", *uri.mScheme);
    EXPECT_EQ(testUri, uri.toString());
  }
}

TEST(UriHelpers, fromString_InputIsUri_DoesNotChange)
{
  Uri uri;

  ASSERT_TRUE(uri.fromString("ftp://ftp.is.co.za/rfc/rfc1808.txt"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  EXPECT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("ftp", *uri.mScheme);
  EXPECT_EQ("ftp.is.co.za", *uri.mAuthority);
  EXPECT_EQ("/rfc/rfc1808.txt", *uri.mPath);
}

TEST(UriHelpers, getUri_InputIsUri_DoesNotChange)
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

  for (const std::string& testUri : testUris) {
    EXPECT_EQ(testUri, Uri::createFromString(testUri).toString());
  }
}

TEST(UriHelpers, getUri_InputIsPath_AppendsFileSchema)
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
    // whereas on Unix systems the additional forward slash is not required
    // since an absolute path already has it.
    const std::string testUri = "file://" + testPath;
#endif
    EXPECT_EQ(testUri, Uri::getUri(testPath));
  }
}

TEST(UriComponent, AccessorsAndDefaults)
{
  UriComponent component;
  EXPECT_FALSE(component);
  EXPECT_TRUE(!component);
  EXPECT_EQ(component.get_value_or("default"), "default");

  component = std::string("value");
  EXPECT_TRUE(component);
  EXPECT_EQ(*component, "value");
  EXPECT_EQ(component.get(), "value");
  EXPECT_EQ(component->size(), std::string("value").size());
}

TEST(UriHelpers, GetPathAndFilesystemPath)
{
  Uri uri = Uri::createFromString("file:///tmp/robot.urdf");
  EXPECT_EQ(uri.getPath(), "/tmp/robot.urdf");
#ifdef _WIN32
  EXPECT_EQ(uri.getFilesystemPath(), "tmp/robot.urdf");
#else
  EXPECT_EQ(uri.getFilesystemPath(), "/tmp/robot.urdf");
#endif
}

TEST(UriHelpers, ResolveRelativeUriRemovesDotSegments)
{
  const std::string base = "http://example.com/a/b/c";
  const std::string relative = "../d/e";
  const auto merged = Uri::createFromRelativeUri(
      std::string_view(base), std::string_view(relative));

  EXPECT_EQ(merged.toString(), "http://example.com/a/d/e");
}

TEST(UriHelpers, fromString_WindowsStyleFileUri_ParsesCorrectly)
{
  Uri uri;

  ASSERT_TRUE(uri.fromString("file:///C:/Users/name/model.urdf"));
  ASSERT_TRUE(uri.mScheme);
  ASSERT_TRUE(uri.mAuthority);
  ASSERT_TRUE(uri.mPath);
  EXPECT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("", *uri.mAuthority);
  EXPECT_EQ("/C:/Users/name/model.urdf", *uri.mPath);

  ASSERT_TRUE(uri.fromString("file:///D:/projects/dart/data/sdf/model.sdf"));
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("", *uri.mAuthority);
  EXPECT_EQ("/D:/projects/dart/data/sdf/model.sdf", *uri.mPath);
}

TEST(UriHelpers, getRelativeUri)
{
  std::vector<std::pair<std::string, std::string>> testPairs = {
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

  Uri baseUri, relativeUri, mergedUri;
  ASSERT_TRUE(baseUri.fromString("http://a/b/c/d;p?q"));

  for (const auto& it : testPairs) {
    const std::string& expectedUri = it.second;

    ASSERT_TRUE(relativeUri.fromString(it.first));

    // Strict mode
    ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, relativeUri, true));
    EXPECT_EQ(expectedUri, mergedUri.toString());

    // Backwards compatibility mode
    ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, relativeUri, false));
    EXPECT_EQ(expectedUri, mergedUri.toString());
  }

  // Strict mode behavior.
  ASSERT_TRUE(relativeUri.fromString("http:g"));
  ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, "http:g", true));
  EXPECT_EQ("http:g", mergedUri.toString());

  // TODO: I'm not sure what we have to do to implement this. The approach
  // described in the RFC doesn't seem to produce the expected result.
#if 0
  // Backwards compatibility mode behavior.
  ASSERT_TRUE(relativeUri.fromString("http:g"));
  ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, "http:g", false));
  EXPECT_EQ("http://a/b/c/g", mergedUri.toString());
#endif
}

TEST(UriHelpers, UriComponentAccessors)
{
  UriComponent component("value");
  std::string fallback = "fallback";

  EXPECT_EQ(component.get_value_or(fallback), "value");
  EXPECT_EQ(component->size(), std::string("value").size());

  UriComponent empty;
  EXPECT_EQ(empty.get_value_or(fallback), fallback);
}

TEST(UriHelpers, CreateFromRelativeUri)
{
  const std::string base = "http://a/b/c/d;p?q";
  const std::string relative = "g";
  const std::string_view baseView(base);
  const std::string_view relativeView(relative);

  const auto mergedFromStrings
      = Uri::createFromRelativeUri(baseView, relativeView, true);
  EXPECT_EQ(mergedFromStrings.toString(), "http://a/b/c/g");

  const auto baseUri = Uri::createFromString(base);
  const auto mergedFromBase
      = Uri::createFromRelativeUri(baseUri, relativeView, true);
  EXPECT_EQ(mergedFromBase.toString(), "http://a/b/c/g");

  const auto relativeUri = Uri::createFromString(relative);
  const auto mergedFromUris
      = Uri::createFromRelativeUri(baseUri, relativeUri, true);
  EXPECT_EQ(mergedFromUris.toString(), "http://a/b/c/g");
}

//==============================================================================
// Edge case tests for improved coverage
//==============================================================================

TEST(UriHelpers, fromString_EmptyString_ReturnsTrue)
{
  Uri uri;
  // Empty string is valid per RFC 3986 (empty relative reference)
  EXPECT_TRUE(uri.fromString(""));
  EXPECT_FALSE(uri.mScheme);
  EXPECT_FALSE(uri.mAuthority);
  EXPECT_TRUE(uri.mPath);
  EXPECT_EQ("", *uri.mPath);
}

TEST(UriHelpers, fromString_PathOnly_NoScheme)
{
  Uri uri;
  // Relative path without scheme
  EXPECT_TRUE(uri.fromString("foo/bar"));
  EXPECT_FALSE(uri.mScheme);
  EXPECT_FALSE(uri.mAuthority);
  EXPECT_TRUE(uri.mPath);
  EXPECT_EQ("foo/bar", *uri.mPath);
}

TEST(UriHelpers, fromString_FragmentOnly)
{
  Uri uri;
  EXPECT_TRUE(uri.fromString("#fragment"));
  EXPECT_FALSE(uri.mScheme);
  EXPECT_FALSE(uri.mAuthority);
  EXPECT_TRUE(uri.mPath);
  EXPECT_EQ("", *uri.mPath);
  EXPECT_TRUE(uri.mFragment);
  EXPECT_EQ("fragment", *uri.mFragment);
}

TEST(UriHelpers, fromString_QueryOnly)
{
  Uri uri;
  EXPECT_TRUE(uri.fromString("?query=value"));
  EXPECT_FALSE(uri.mScheme);
  EXPECT_FALSE(uri.mAuthority);
  EXPECT_TRUE(uri.mPath);
  EXPECT_EQ("", *uri.mPath);
  EXPECT_TRUE(uri.mQuery);
  EXPECT_EQ("query=value", *uri.mQuery);
}

TEST(UriHelpers, fromString_EmptyAuthority)
{
  Uri uri;
  EXPECT_TRUE(uri.fromString("file:///path/to/file"));
  EXPECT_TRUE(uri.mScheme);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_TRUE(uri.mAuthority);
  EXPECT_EQ("", *uri.mAuthority);
  EXPECT_TRUE(uri.mPath);
  EXPECT_EQ("/path/to/file", *uri.mPath);
}

TEST(UriHelpers, merge_EmptyBasePath_WithAuthority)
{
  Uri baseUri, relativeUri, mergedUri;
  // Base with authority but empty path
  ASSERT_TRUE(baseUri.fromString("http://example.com"));
  ASSERT_TRUE(relativeUri.fromString("relative"));
  ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, relativeUri, true));
  EXPECT_EQ("http://example.com/relative", mergedUri.toString());
}

TEST(UriHelpers, merge_RootPath)
{
  Uri baseUri, relativeUri, mergedUri;
  // Base with root path only
  ASSERT_TRUE(baseUri.fromString("http://example.com/"));
  ASSERT_TRUE(relativeUri.fromString("relative"));
  ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, relativeUri, true));
  EXPECT_EQ("http://example.com/relative", mergedUri.toString());
}

TEST(UriHelpers, merge_RelativeWithAbsolutePath)
{
  Uri baseUri, relativeUri, mergedUri;
  ASSERT_TRUE(baseUri.fromString("http://a/b/c/d"));
  ASSERT_TRUE(relativeUri.fromString("/absolute"));
  ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, relativeUri, true));
  EXPECT_EQ("http://a/absolute", mergedUri.toString());
}

TEST(UriHelpers, merge_RelativeWithAuthority)
{
  Uri baseUri, relativeUri, mergedUri;
  ASSERT_TRUE(baseUri.fromString("http://a/b/c/d"));
  ASSERT_TRUE(relativeUri.fromString("//newhost/path"));
  ASSERT_TRUE(mergedUri.fromRelativeUri(baseUri, relativeUri, true));
  EXPECT_EQ("http://newhost/path", mergedUri.toString());
}

TEST(UriHelpers, getFilesystemPath_FileUri)
{
  Uri uri;
  ASSERT_TRUE(uri.fromString("file:///home/user/file.txt"));
  // On Unix, getFilesystemPath returns the path as-is
#ifndef _WIN32
  EXPECT_EQ("/home/user/file.txt", uri.getFilesystemPath());
#endif
}

TEST(UriHelpers, getFilesystemPath_NonFileUri)
{
  Uri uri;
  ASSERT_TRUE(uri.fromString("http://example.com/path"));
  // For non-file URIs, getFilesystemPath returns the path component
  EXPECT_EQ("/path", uri.getFilesystemPath());
}

TEST(UriHelpers, clear_ResetsAllComponents)
{
  Uri uri;
  ASSERT_TRUE(uri.fromString("http://user@host:8080/path?query#fragment"));
  EXPECT_TRUE(uri.mScheme);
  EXPECT_TRUE(uri.mAuthority);
  EXPECT_TRUE(uri.mPath);
  EXPECT_TRUE(uri.mQuery);
  EXPECT_TRUE(uri.mFragment);

  uri.clear();

  EXPECT_FALSE(uri.mScheme);
  EXPECT_FALSE(uri.mAuthority);
  EXPECT_FALSE(uri.mPath);
  EXPECT_FALSE(uri.mQuery);
  EXPECT_FALSE(uri.mFragment);
}

TEST(UriHelpers, toString_PartialComponents)
{
  Uri uri;
  // URI with only scheme and path
  ASSERT_TRUE(uri.fromString("mailto:user@example.com"));
  EXPECT_EQ("mailto:user@example.com", uri.toString());

  // URI with query and fragment
  ASSERT_TRUE(uri.fromString("http://host/path?q=1#frag"));
  EXPECT_EQ("http://host/path?q=1#frag", uri.toString());
}

TEST(UriHelpers, getUri_InvalidInput_ReturnsEmpty)
{
  // getUri with empty string should still work (empty is valid)
  std::string result = Uri::getUri("");
#ifndef _WIN32
  // On Unix, empty string is treated as path and becomes file://
  EXPECT_EQ("file://", result);
#endif
}

TEST(UriHelpers, getRelativeUri_WithBaseUri)
{
  Uri baseUri;
  ASSERT_TRUE(baseUri.fromString("http://a/b/c"));
  std::string_view relative = "relative";
  std::string result = Uri::getRelativeUri(baseUri, relative, true);
  EXPECT_EQ("http://a/b/relative", result);
}

TEST(UriHelpers, createFromPath_AbsolutePath)
{
#ifdef _WIN32
  const std::string path = "C:\\Users\\test\\file.txt";
#else
  const std::string path = "/home/user/file.txt";
#endif
  Uri uri = Uri::createFromPath(path);
  EXPECT_TRUE(uri.mScheme);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_TRUE(uri.mAuthority);
  EXPECT_EQ("", *uri.mAuthority);
  EXPECT_EQ(path, uri.getFilesystemPath());
}

TEST(UriHelpers, createFromStringOrPath_WithUri)
{
  Uri uri = Uri::createFromStringOrPath("http://example.com/path");
  EXPECT_TRUE(uri.mScheme);
  EXPECT_EQ("http", *uri.mScheme);
  EXPECT_EQ("http://example.com/path", uri.toString());
}

TEST(UriHelpers, createFromStringOrPath_WithPath)
{
#ifndef _WIN32
  Uri uri = Uri::createFromStringOrPath("/absolute/path");
  EXPECT_TRUE(uri.mScheme);
  EXPECT_EQ("file", *uri.mScheme);
  EXPECT_EQ("/absolute/path", uri.getFilesystemPath());
#endif
}

TEST(UriHelpers, getRelativeUri_WithUriObjects)
{
  Uri baseUri = Uri::createFromString("http://a/b/c/d");
  Uri relativeUri = Uri::createFromString("../e");
  std::string result = Uri::getRelativeUri(baseUri, relativeUri, true);
  EXPECT_EQ("http://a/b/e", result);
}

TEST(UriHelpers, fromRelativeUri_CharPtrOverload)
{
  Uri mergedUri;
  ASSERT_TRUE(mergedUri.fromRelativeUri("http://a/b/c", "g", true));
  EXPECT_EQ("http://a/b/g", mergedUri.toString());
}

TEST(UriHelpers, UriComponent_MutableAccessors)
{
  UriComponent component("initial");
  EXPECT_EQ(*component, "initial");

  *component = "modified";
  EXPECT_EQ(*component, "modified");

  component->clear();
  EXPECT_TRUE(component->empty());
}

TEST(UriHelpers, UriComponent_ArrowOperator)
{
  UriComponent component("test_value");
  EXPECT_EQ(component->length(), 10u);
  EXPECT_EQ(component->substr(0, 4), "test");
}

TEST(UriHelpers, getPath_ReturnsPathComponent)
{
  Uri uri;
  ASSERT_TRUE(uri.fromString("http://host/path/to/resource?query"));
  EXPECT_EQ("/path/to/resource", uri.getPath());
}

TEST(UriHelpers, getPath_EmptyPath)
{
  Uri uri;
  ASSERT_TRUE(uri.fromString("http://host"));
  EXPECT_EQ("", uri.getPath());
}

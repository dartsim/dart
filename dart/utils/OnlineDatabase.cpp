/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/utils/OnlineDatabase.hpp"

#include <cstdlib>

#include <boost/filesystem.hpp>
#include <curl/curl.h>
#include <tinyxml2.h>

namespace dart {
namespace utils {

#define DART_DATABASE_URI                                                      \
  "https://raw.githubusercontent.com/dartsim/data/master/"
// TODO(JS): Set correctly

#define GZ_MODEL_MANIFEST_FILENAME "dart_manifest.xml"

namespace {

//==============================================================================
std::size_t writeData(
    void* ptr, std::size_t size, std::size_t nmemb, std::FILE* stream)
{
  std::size_t written;
  written = std::fwrite(ptr, size, nmemb, stream);
  return written;
}

//==============================================================================
std::size_t getModelsCallback(
    void* buffer, std::size_t size, std::size_t nmemb, void* userp)
{
  std::string* str = static_cast<std::string*>(userp);
  size *= nmemb;

  // Append the new character data to the string
  str->append(static_cast<const char*>(buffer), size);
  return size;
}

} // (anonymous) namespace

//==============================================================================
void OnlineDatabase::startup(bool fetchImmediately)
{
  std::lock_guard<std::recursive_mutex> lock(startCacheMutex);
  DART_UNUSED(lock);

  if (!updateCacheThread)
  {
    stop = false;

    // Create the thread that is used to update the model cache. This retreives
    // online data in the background to improve startup times.
    updateCacheThread = new std::thread(
        std::bind(&OnlineDatabase::UpdateModelCache, this, fetchImmediately));
  }
}

//==============================================================================
void OnlineDatabase::shutdown()
{
  stop = true;
  updateCacheCompleteCondition.notify_all();
  updateCacheCondition.notify_all();

  {
    std::lock_guard<std::recursive_mutex> lock(startCacheMutex);
    DART_UNUSED(lock);

    if (updateCacheThread)
      updateCacheThread->join();
    delete updateCacheThread;
    updateCacheThread = nullptr;
  }
}

//==============================================================================
std::string OnlineDatabase::getUri()
{
  std::string result;
  if (const char* uriStr = std::getenv("DART_DATABASE_URI"))
    result = uriStr;
  else
    result = DART_DATABASE_URI; // No env var. Take compile-time default.

  if (result[result.size() - 1] != '/')
    result += '/';

  return result;
}

//==============================================================================
bool OnlineDatabase::hasData(const common::Uri& dataUri)
{
  //  std::string uri = dataUri;

  //  std::replace_first(uri, "model://", getUri());
  //  uri = uri.substr(0, uri.find("/", getUri().size()));

  //  const auto models = getModels();

  //  for (auto iter = models.begin(); iter != models.end(); ++iter)
  //  {
  //    if (iter->first == uri)
  //      return true;
  //  }

  return false;
}

//==============================================================================
std::map<common::Uri, std::string> OnlineDatabase::getModels()
{
  std::size_t size = 0u;

  {
    std::lock_guard<std::recursive_mutex> startLock(startCacheMutex);
    DART_UNUSED(startLock);

    if (!updateCacheThread)
    {
      std::unique_lock<std::mutex> lock(updateMutex);
      startup(true);
      updateCacheCompleteCondition.wait(lock);
    }
    else
    {
      std::unique_lock<std::mutex> lock(updateMutex, std::try_to_lock);
      if (!lock.owns_lock())
      {
        dtmsg << "Waiting for model database update to complete...\n";
        std::lock_guard<std::mutex> lock2(updateMutex);
        DART_UNUSED(lock2);
      }
    }

    size = modelCache.size();
  }

  if (size != 0u)
  {
    return modelCache;
  }
  else
  {
    dtwarn << "Getting models from[" << getUri()
           << "]. This may take a few seconds.\n";

    std::unique_lock<std::mutex> lock(updateMutex);

    // Tell the background thread to grab the models from online.
    updateCacheCondition.notify_all();

    // Wait for the thread to finish.
    updateCacheCompleteCondition.wait(lock);
  }

  return modelCache;
}

//==============================================================================
std::string OnlineDatabase::GetModelConfig(const std::string& modelUri)
{
  std::string xmlString;
  //  std::string uri = modelUri;
  //  std::replace_first(uri, "model://", OnlineDatabase::GetURI());

  //  if (!uri.empty())
  //  {
  //    std::string manifestURI = uri + "/" + GZ_MODEL_MANIFEST_FILENAME;
  //    xmlString = this->GetManifestImpl(manifestURI);
  //  }

  return xmlString;
}

//==============================================================================
std::string OnlineDatabase::GetDBConfig(const std::string& configUri)
{
  std::string xmlString;
  //  std::string uri = configUri;
  //  std::replace_first(uri, "model://", getUri());

  //  if (!uri.empty())
  //  {
  //    std::string manifestURI = uri + "/" + GZ_MODEL_DB_MANIFEST_FILENAME;
  //    xmlString = this->GetManifestImpl(manifestURI);
  //  }

  return xmlString;
}

//==============================================================================
std::string OnlineDatabase::GetModelPath(const std::string& uri, bool forceDownload)
{

}

//==============================================================================
std::string OnlineDatabase::GetModelFile(const std::string& uri)
{
  std::string result;

  // This will download the model if necessary
  std::string path = GetModelPath(uri);

  boost::filesystem::path manifestPath = path;

  // Get the GZ_MODEL_MANIFEST_FILENAME.
  if (boost::filesystem::exists(manifestPath / GZ_MODEL_MANIFEST_FILENAME))
    manifestPath /= GZ_MODEL_MANIFEST_FILENAME;
  else
  {
    dterr << "Missing " << GZ_MODEL_MANIFEST_FILENAME
      << " for model " << manifestPath << "\n";
  }

  tinyxml2::XMLDocument xmlDoc;
  common::Version sdfParserVersion(SDF_VERSION);
  std::string bestVersionStr = "0.0";
  if (xmlDoc.LoadFile(manifestPath.string()))
  {
    tinyxml2::XMLElement *modelXML = xmlDoc.FirstChildElement("model");
    if (modelXML)
    {
      tinyxml2::XMLElement *sdfXML = modelXML->FirstChildElement("sdf");
      tinyxml2::XMLElement *sdfSearch = sdfXML;

      // Find the SDF element that matches our current SDF version.
      // If a match is not found, use the latest version of the element
      // that is not older than the SDF parser.
      while (sdfSearch)
      {
        if (sdfSearch->Attribute("version"))
        {
          std::string version = std::string(sdfSearch->Attribute("version"));
          common::Version modelVersion(version);
          common::Version bestVersion(bestVersionStr);
          if (modelVersion > bestVersion)
          {
            // this model is better than the previous one
            if (modelVersion <= sdfParserVersion)
            {
              // the parser can read it
              sdfXML = sdfSearch;
              bestVersionStr = version;
            }
            else
            {
              dtwarn << "Ignoring version " << version
                << " for model " << uri
                << " because Gazebo is using an older sdf parser (version "
                << SDF_VERSION << ")" << std::endl;
            }
          }
        }
        sdfSearch = sdfSearch->NextSiblingElement("sdf");
      }

      if (sdfXML)
      {
        result = path + "/" + sdfXML->GetText();
      }
      else
      {
        dterr << "Manifest[" << manifestPath << "] doesn't have "
              << "<model><sdf>...</sdf></model> element.\n";
      }
    }
    else
    {
      dterr << "Manifest[" << manifestPath
            << "] doesn't have a <model> element\n";
    }
  }
  else
  {
    dterr << "Invalid model manifest file[" << manifestPath << "]\n";
  }

  return result;
}

//==============================================================================
OnlineDatabase::OnlineDatabase() : updateCacheThread(nullptr)
{
  startup();
}

//==============================================================================
OnlineDatabase::~OnlineDatabase()
{
  shutdown();
}

//==============================================================================
std::string OnlineDatabase::GetManifestImpl(const std::string& uri)
{
  std::string xmlString;
  if (!uri.empty())
  {
    CURL* curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, uri.c_str());

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, getModelsCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &xmlString);

    CURLcode success = curl_easy_perform(curl);
    if (success != CURLE_OK)
    {
      dtwarn << "Unable to connect to model database using [" << uri
             << "]. Only locally installed models will be available.\n";
    }

    curl_easy_cleanup(curl);
  }

  return xmlString;
}

//==============================================================================
void OnlineDatabase::UpdateModelCache(bool fetchImmediately)
{
}

} // namespace utils
} // namespace dart

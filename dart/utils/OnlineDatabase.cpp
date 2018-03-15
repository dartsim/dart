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

#include "dart/common/Platform.hpp"
#include "dart/common/Version.hpp"

namespace dart {
namespace utils {

#define DART_DATABASE_URI                                                      \
  "https://raw.githubusercontent.com/dartsim/data/master/"
// TODO(JS): Set correctly

#define GZ_MODEL_MANIFEST_FILENAME "config.xml" // TODO: Change to yaml file
#define SDF_VERSION "0.0.1"
#define GZ_MODEL_DB_MANIFEST_FILENAME "" // TODO: Define

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
std::map<std::string, std::string> OnlineDatabase::getModels()
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
std::string OnlineDatabase::GetModelName(const std::string& uri)
{
  const std::string xmlStr = GetModelConfig(uri);

  if (xmlStr.empty())
  {
    dterr << "Unable to get model name [" << uri << "]\n";
    return "";
  }

  tinyxml2::XMLDocument xmlDoc;
  if (!xmlDoc.Parse(xmlStr.c_str()))
  {
    dterr << "Unable to parse " << GZ_MODEL_MANIFEST_FILENAME << " for model ["
          << uri << "].\n";
    return "";
  }

  const auto* modelElem = xmlDoc.FirstChildElement("model");
  if (!modelElem)
  {
    dterr << "No <model> element in " << GZ_MODEL_MANIFEST_FILENAME
          << " for model [" << uri << "].\n";
    return "";
  }

  const auto* nameElem = modelElem->FirstChildElement("name");
  if (!nameElem)
  {
    dterr << "No <name> element in " << GZ_MODEL_MANIFEST_FILENAME
          << " for model [" << uri << "].\n";
    return "";
  }

  return nameElem->GetText();
}

//==============================================================================
std::string OnlineDatabase::GetModelConfig(const std::string& modelUri)
{
  std::string xmlString;
  //  std::string uri = modelUri;
  //  std::replace_first(uri, "model://", getUri());

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
std::string OnlineDatabase::GetModelPath(
    const std::string& uri, bool forceDownload)
{
  //  std::string path, suffix;

  //  if (!forceDownload)
  //    path = SystemPaths::Instance()->FindFileURI(uri);

  //  struct stat st;

  //  if (path.empty() || stat(path.c_str(), &st) != 0)
  //  {
  //    if (!HasModel(uri))
  //    {
  //      dterr << "Unable to download model[" << uri << "]\n";
  //      return std::string();
  //    }

  //    // DEBUG output
  //    // std::cout << "Getting uri[" << _uri << "] path[" << path << "]\n";

  //    // Get the model name from the uri
  //    size_t startIndex = uri.find_first_of("://");
  //    if (startIndex == std::string::npos)
  //    {
  //      dterr << "URI[" << uri << "] is missing ://\n";
  //      return std::string();
  //    }

  //    std::string modelName = uri;
  //    boost::replace_first(modelName, "model://", "");
  //    boost::replace_first(modelName, getUri(), "");

  //    startIndex = modelName[0] == '/' ? 1 : 0;
  //    size_t endIndex = modelName.find_first_of("/", startIndex);
  //    size_t modelNameLen = endIndex == std::string::npos ? std::string::npos
  //                                                        : endIndex -
  //                                                        startIndex;

  //    if (endIndex != std::string::npos)
  //      suffix = modelName.substr(endIndex, std::string::npos);

  //    modelName = modelName.substr(startIndex, modelNameLen);

  //    // Store downloaded .tar.gz and intermediate .tar files in temp location
  //    boost::filesystem::path tmppath =
  //    boost::filesystem::temp_directory_path();
  //    tmppath /=
  //    boost::filesystem::unique_path("gz_model-%%%%-%%%%-%%%%-%%%%");
  //    std::string tarfilename = tmppath.string() + ".tar";
  //    std::string tgzfilename = tarfilename + ".gz";

  //    CURL* curl = curl_easy_init();
  //    if (!curl)
  //    {
  //      dterr << "Unable to initialize libcurl\n";
  //      return std::string();
  //    }

  //    curl_easy_setopt(
  //        curl,
  //        CURLOPT_URL,
  //        (getUri() + "/" + modelName + "/model.tar.gz").c_str());
  //    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeData);

  //    bool retry = true;
  //    int iterations = 0;
  //    while (retry && iterations < 4)
  //    {
  //      retry = false;
  //      iterations++;

  //      FILE* fp = fopen(tgzfilename.c_str(), "wb");
  //      if (!fp)
  //      {
  //        dterr << "Could not download model[" << uri << "] because we were"
  //              << "unable to write to file[" << tgzfilename << "]."
  //              << "Please fix file permissions.";
  //        return std::string();
  //      }

  //      /// Download the model tarball
  //      curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
  //      CURLcode success = curl_easy_perform(curl);

  //      if (success != CURLE_OK)
  //      {
  //        dtwarn << "Unable to connect to model database using [" << uri <<
  //        "]\n";
  //        retry = true;
  //        continue;
  //      }

  //      std::fclose(fp);

  //      try
  //      {
  //        // Unzip model tarball
  //        std::ifstream file(
  //            tgzfilename.c_str(), std::ios_base::in | std::ios_base::binary);
  //        std::ofstream out(
  //            tarfilename.c_str(), std::ios_base::out |
  //            std::ios_base::binary);
  //        boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
  //        in.push(boost::iostreams::gzip_decompressor());
  //        in.push(file);
  //        boost::iostreams::copy(in, out);
  //      }
  //      catch (...)
  //      {
  //        dterr << "Failed to unzip model tarball. Trying again...\n";
  //        retry = true;
  //        continue;
  //      }

  ////#if !DART_OS_WINDOWS
  ////      TAR* tar;
  ////      tar_open(
  ////          &tar,
  ////          const_cast<char*>(tarfilename.c_str()),
  ////          nullptr,
  ////          O_RDONLY,
  ////          0644,
  ////          TAR_GNU);

  ////      std::string outputPath = getenv("HOME");
  ////      outputPath += "/.gazebo/models";

  ////      tar_extract_all(tar, const_cast<char*>(outputPath.c_str()));
  ////      path = outputPath + "/" + modelName;

  ////      DownloadDependencies(path);
  ////#endif
  //    }

  //    curl_easy_cleanup(curl);
  //    if (retry)
  //    {
  //      dterr << "Could not download model[" << uri << "]."
  //            << "The model may be corrupt.\n";
  //      path.clear();
  //    }

  //    // Clean up
  //    try
  //    {
  //      boost::filesystem::remove(tarfilename);
  //      boost::filesystem::remove(tgzfilename);
  //    }
  //    catch (...)
  //    {
  //      dtwarn << "Failed to remove temporary model files after download.";
  //    }
  //  }

  //  return path + suffix;
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
  {
    manifestPath /= GZ_MODEL_MANIFEST_FILENAME;
  }
  else
  {
    dterr << "Missing " << GZ_MODEL_MANIFEST_FILENAME << " for model "
          << manifestPath << "\n";
  }

  tinyxml2::XMLDocument xmlDoc;
  common::Version sdfParserVersion(SDF_VERSION);
  std::string bestVersionStr = "0.0";
  if (xmlDoc.LoadFile(manifestPath.string().c_str()))
  {
    tinyxml2::XMLElement* modelXML = xmlDoc.FirstChildElement("model");
    if (modelXML)
    {
      tinyxml2::XMLElement* sdfXML = modelXML->FirstChildElement("sdf");
      tinyxml2::XMLElement* sdfSearch = sdfXML;

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
              dtwarn << "Ignoring version " << version << " for model " << uri
                     << " because Gazebo is using an older sdf parser (version "
                     << SDF_VERSION << ")\n";
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
void OnlineDatabase::DownloadDependencies(const std::string& path)
{
}

//==============================================================================
bool OnlineDatabase::HasModel(const std::string& modelName)
{
  return false;
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
  std::unique_lock<std::mutex> lock(updateMutex);

  // Continually update the model cache when requested.
  while (!stop)
  {
    if (fetchImmediately)
      fetchImmediately = false;
    else // Wait for an update request.
      updateCacheCondition.wait(lock);

    // Exit if notified and stopped.
    if (stop)
      break;

    // Update the model cache.
    if (UpdateModelCacheImpl())
    {
      std::lock_guard<std::mutex> lock2(callbacksMutex);
      DART_UNUSED(lock2);

      if (stop)
        break;

      //      modelDBUpdated(modelCache); // TODO: Change to signal
    }
    else
    {
      dterr << "Unable to download model manifests\n";
    }

    updateCacheCompleteCondition.notify_all();
  }

  // Make sure no one is waiting on us.
  updateCacheCompleteCondition.notify_all();
}

//==============================================================================
bool OnlineDatabase::UpdateModelCacheImpl()
{
  std::string xmlString = GetDBConfig(getUri());

  if (!xmlString.empty())
  {
    tinyxml2::XMLDocument xmlDoc;
    xmlDoc.Parse(xmlString.c_str());

    auto* databaseElem = xmlDoc.FirstChildElement("database");
    if (!databaseElem)
    {
      dterr << "No <database> tag in the model database "
            << GZ_MODEL_DB_MANIFEST_FILENAME << " found"
            << " here[" << getUri() << "].\n";
      return false;
    }

    auto* modelsElem = databaseElem->FirstChildElement("models");
    if (!modelsElem)
    {
      dterr << "No <models> tag in the model database "
            << GZ_MODEL_DB_MANIFEST_FILENAME << " found"
            << " here[" << getUri() << "].\n";
      return false;
    }

    tinyxml2::XMLElement* uriElem;
    for (uriElem = modelsElem->FirstChildElement("uri");
         uriElem != nullptr && !stop;
         uriElem = uriElem->NextSiblingElement("uri"))
    {
      std::string uri = uriElem->GetText();

      size_t index = uri.find("://");
      std::string suffix = uri;
      if (index != std::string::npos)
      {
        suffix = uri.substr(index + 3, uri.size() - index - 3);
      }

      std::string fullURI = getUri() + suffix;
      std::string modelName = GetModelName(fullURI);

      modelCache[fullURI] = modelName;
    }
  }

  return true;
}

} // namespace utils
} // namespace dart

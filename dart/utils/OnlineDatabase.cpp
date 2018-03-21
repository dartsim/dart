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
#include <fcntl.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <curl/curl.h>
#include <tinyxml2.h>

#if !DART_OS_WINDOWS
#include <libtar.h>
#endif

#include "dart/common/Platform.hpp"
#include "dart/common/Version.hpp"

// TODO:
// - Replace XML with YAML
// - Rename Model to Skeleton
// - Support other skeleton file formats such as Skel, URDF, and VSK.
// - Use ResourceRetriever
// - Fix const correctness

namespace dart {
namespace utils {

#define SDF_VERSION "0.0.1"

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
OnlineDatabase::OnlineDatabase(
    const std::string& serverUri,
    const std::string& baseUri,
    const std::string& databaseLocalPath,
    const std::string& databaseConfigFilename,
    const std::string& datasetConfigFilename)
  : mServerUri(serverUri),
    mBaseUri(baseUri),
    mDatabaseLocalpath(databaseLocalPath),
    mDatabaseConfigFilename(databaseConfigFilename),
    mDatasetConfigFilename(datasetConfigFilename),
    mUpdateDatasetMapThread(nullptr)
{
  startup();
}

//==============================================================================
OnlineDatabase::~OnlineDatabase()
{
  shutdown();
}

//==============================================================================
void OnlineDatabase::startup(bool fetchImmediately)
{
  std::lock_guard<std::recursive_mutex> lock(mUpdateDatasetThreadMutex);
  DART_UNUSED(lock);

  if (!mUpdateDatasetMapThread)
  {
    mStop = false;

    // Create the thread that is used to update the model cache. This retreives
    // online data in the background to improve startup times.
    mUpdateDatasetMapThread = new std::thread(
        std::bind(&OnlineDatabase::updateDatasetMap, this, fetchImmediately));
  }
}

//==============================================================================
void OnlineDatabase::shutdown()
{
  mStop = true;
  mUpdateDatasetMapCompleteCondition.notify_all();
  mUpdateCacheCondition.notify_all();

  {
    std::lock_guard<std::recursive_mutex> lock(mUpdateDatasetThreadMutex);
    DART_UNUSED(lock);

    if (mUpdateDatasetMapThread)
      mUpdateDatasetMapThread->join();

    delete mUpdateDatasetMapThread;

    mUpdateDatasetMapThread = nullptr;
  }
}

//==============================================================================
std::string OnlineDatabase::getServerUri()
{
  return mServerUri;
}

//==============================================================================
const std::map<std::string, std::string>& OnlineDatabase::getDatasets()
{
  std::size_t size = 0u;

  {
    std::lock_guard<std::recursive_mutex> startLock(mUpdateDatasetThreadMutex);
    DART_UNUSED(startLock);

    if (mUpdateDatasetMapThread)
    {
      std::unique_lock<std::mutex> lock(mDatasetMapMutex, std::try_to_lock);
      if (!lock.owns_lock())
      {
        dtmsg << "Waiting for model database update to complete...\n";
        std::lock_guard<std::mutex> lock2(mDatasetMapMutex);
        DART_UNUSED(lock2);
      }
    }
    else
    {
      std::unique_lock<std::mutex> lock(mDatasetMapMutex);
      startup(true);
      mUpdateDatasetMapCompleteCondition.wait(lock);
    }

    size = mDatasetMap.size();
  }

  if (size != 0u)
  {
    return mDatasetMap;
  }
  else
  {
    dtwarn << "Getting models from[" << getServerUri()
           << "]. This may take a few seconds.\n";

    std::unique_lock<std::mutex> lock(mDatasetMapMutex);

    // Tell the background thread to grab the models from online.
    mUpdateCacheCondition.notify_all();

    // Wait for the thread to finish.
    mUpdateDatasetMapCompleteCondition.wait(lock);
  }

  return mDatasetMap;
}

//==============================================================================
std::string OnlineDatabase::getDatasetName(const std::string& uri)
{
  const std::string xmlStr = getDatasetConfig(uri);

  if (xmlStr.empty())
  {
    dterr << "Unable to get model name [" << uri << "]\n";
    return "";
  }

  tinyxml2::XMLDocument xmlDoc;
  auto result = xmlDoc.Parse(xmlStr.c_str());
  if (tinyxml2::XML_SUCCESS != result)
  {
    dterr << "Unable to parse " << mDatasetConfigFilename << " for model ["
          << uri << "].\n";
    return "";
  }

  const auto* modelElem = xmlDoc.FirstChildElement("model");
  if (!modelElem)
  {
    dterr << "No <model> element in " << mDatasetConfigFilename
          << " for model [" << uri << "].\n";
    return "";
  }

  const auto* nameElem = modelElem->FirstChildElement("name");
  if (!nameElem)
  {
    dterr << "No <name> element in " << mDatasetConfigFilename << " for model ["
          << uri << "].\n";
    return "";
  }

  return nameElem->GetText();
}

//==============================================================================
std::string OnlineDatabase::getDatasetConfig(const std::string& datasetUri)
{
  std::string xmlString;
  std::string uri = datasetUri;
  boost::replace_first(uri, mBaseUri, getServerUri());

  if (!uri.empty())
  {
    std::string manifestURI = uri + "/" + mDatasetConfigFilename;
    xmlString = getManifestImpl(manifestURI);
  }

  return xmlString;
}

//==============================================================================
std::string OnlineDatabase::getDatabaseConfig(const std::string& configUri)
{
  std::string uri = configUri;

  boost::replace_first(uri, mBaseUri, getServerUri());

  if (uri.empty())
    return "";

  const std::string manifestUri = uri + "/" + mDatabaseConfigFilename;

  return getManifestImpl(manifestUri);
}

//==============================================================================
std::string OnlineDatabase::getDatasetPath(std::string uri, bool forceDownload)
{
  std::string datasetName = uri;
  boost::replace_first(datasetName, mBaseUri, "");
  boost::replace_first(datasetName, getServerUri(), "");



  // DEBUG output
  // std::cout << "Getting uri[" << _uri << "] path[" << path << "]\n";

  // Get the model name from the uri
  std::size_t startIndex = uri.find_first_of("://");
  if (startIndex == std::string::npos)
  {
    dterr << "URI[" << uri << "] is missing ://\n";
    return std::string();
  }

  std::string modelName = uri;
  boost::replace_first(modelName, mBaseUri, "");
  boost::replace_first(modelName, getServerUri(), "");

  startIndex = modelName[0] == '/' ? 1 : 0;
  std::size_t endIndex = modelName.find_first_of("/", startIndex);
  std::size_t modelNameLen = endIndex == std::string::npos
                                 ? std::string::npos
                                 : endIndex - startIndex;

  std::string suffix;
  if (endIndex != std::string::npos)
    suffix = modelName.substr(endIndex, std::string::npos);

  modelName = modelName.substr(startIndex, modelNameLen);




  std::string datasetPath;
  datasetPath = mDatabaseLocalpath + "/" + modelName;





  auto datasetPath2 = resolveDatasetAndData(uri).first;
  auto path = resolveDatasetAndData(uri).second;

  uri = mBaseUri + datasetPath2;

  auto exists = boost::filesystem::is_directory(datasetPath);

  if (!exists || forceDownload)
  {
    if (!hasDataset(uri))
    {
      dterr << "Unable to download model[" << uri << "]\n";
      return std::string();
    }



    // Store downloaded .tar.gz and intermediate .tar files in temp location
    boost::filesystem::path tmppath = boost::filesystem::temp_directory_path();
    tmppath /= boost::filesystem::unique_path("gz_model-%%%%-%%%%-%%%%-%%%%");
    std::string tarfilename = tmppath.string() + ".tar";
    std::string tgzfilename = tarfilename + ".gz";

    CURL* curl = curl_easy_init();
    if (!curl)
    {
      dterr << "Unable to initialize libcurl\n";
      return std::string();
    }

    curl_easy_setopt(
        curl,
        CURLOPT_URL,
        (getServerUri() + "/" + modelName + "/model.tar.gz").c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeData);

    bool retry = true;
    int iterations = 0;
    while (retry && iterations < 4)
    {
      retry = false;
      iterations++;

      FILE* fp = fopen(tgzfilename.c_str(), "wb");
      if (!fp)
      {
        dterr << "Could not download model[" << uri << "] because we were"
              << "unable to write to file[" << tgzfilename << "]."
              << "Please fix file permissions.";
        return std::string();
      }

      /// Download the model tarball
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
      CURLcode success = curl_easy_perform(curl);

      if (success != CURLE_OK)
      {
        dtwarn << "Unable to connect to model database using [" << uri << "]\n";
        retry = true;
        continue;
      }

      std::fclose(fp);

      try
      {
        // Unzip model tarball
        std::ifstream file(
            tgzfilename.c_str(), std::ios_base::in | std::ios_base::binary);
        std::ofstream out(
            tarfilename.c_str(), std::ios_base::out | std::ios_base::binary);
        boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
        in.push(boost::iostreams::gzip_decompressor());
        in.push(file);
        boost::iostreams::copy(in, out);
      }
      catch (...)
      {
        dterr << "Failed to unzip model tarball. Trying again...\n";
        retry = true;
        continue;
      }

#if !DART_OS_WINDOWS
      TAR* tar;
      tar_open(
          &tar,
          const_cast<char*>(tarfilename.c_str()),
          nullptr,
          O_RDONLY,
          0644,
          TAR_GNU);

      tar_extract_all(tar, const_cast<char*>(mDatabaseLocalpath.c_str()));

      downloadDependencies(datasetPath);
#endif
    }

    curl_easy_cleanup(curl);
    if (retry)
    {
      dterr << "Could not download model[" << uri << "]."
            << "The model may be corrupt.\n";
      datasetPath.clear();
    }

    // Clean up
    try
    {
      boost::filesystem::remove(tarfilename);
      boost::filesystem::remove(tgzfilename);
    }
    catch (...)
    {
      dtwarn << "Failed to remove temporary model files after download.";
    }
  }

  return datasetPath + suffix;
}

//==============================================================================
std::string OnlineDatabase::getDataPath(const std::string& uri)
{
  std::string result;

  // This will download the model if necessary
  std::string path = getDatasetPath(uri);

  boost::filesystem::path manifestPath = path;

  result = (manifestPath / resolveDatasetAndData(uri).second).string();

  // Get the mDatasetConfigFilename.
  if (!boost::filesystem::exists(result))
  {
    dterr << "Missing " << result << ".\n";
    return "";
  }

  return result;
}

//==============================================================================
void OnlineDatabase::downloadDependencies(const std::string& path)
{
  boost::filesystem::path manifestPath = path;

  // Get the mDatasetConfigFilename.
  if (boost::filesystem::exists(manifestPath / mDatasetConfigFilename))
  {
    manifestPath /= mDatasetConfigFilename;
  }
  else
  {
    dterr << "Missing " << mDatasetConfigFilename << " for model " << path
          << "\n";
  }

  tinyxml2::XMLDocument xmlDoc;
  const auto result = xmlDoc.LoadFile(manifestPath.string().c_str());
  if (tinyxml2::XML_SUCCESS != result)
  {
    dterr << "Unable to load manifest file[" << manifestPath << "]\n";
    return;
  }

  auto* modelXML = xmlDoc.FirstChildElement("model");
  if (!modelXML)
  {
    dterr << "No <model> element in manifest file[" << path << "]\n";
    return;
  }

  auto* dependXml = modelXML->FirstChildElement("depend");
  if (!dependXml)
    return;

  for (auto* depXML = dependXml->FirstChildElement("model"); depXML;
       depXML = depXML->NextSiblingElement())
  {
    const auto* uriXml = depXML->FirstChildElement("uri");
    if (uriXml)
    {
      // Download the model if it doesn't exist.
      getDatasetPath(uriXml->GetText());
    }
    else
    {
      dterr << "Model depend is missing <uri> in manifest[" << manifestPath
            << "].\n";
    }
  }
}

//==============================================================================
bool OnlineDatabase::hasDataset(const std::string& datasetUri)
{
  std::string uri = mBaseUri + resolveDatasetAndData(datasetUri).first;

  std::size_t uriSeparator = uri.find("://");

  // Make sure there is a URI separator
  if (uriSeparator == std::string::npos)
  {
    dterr << "No URI separator \"://\" in [" << datasetUri << "].\n";
    return false;
  }

  boost::replace_first(uri, mBaseUri, getServerUri());
  uri = uri.substr(0, uri.find("/", getServerUri().size()));

  const auto models = getDatasets();

  for (auto iter = models.begin(); iter != models.end(); ++iter)
  {
    if (iter->first == uri)
      return true;
  }

  return false;
}

//==============================================================================
std::pair<std::string, std::string> OnlineDatabase::resolveDatasetAndData(
    const std::string& fullUri)
{
  std::string uri = fullUri;

  auto path = uri.substr(mBaseUri.size(), uri.size() - mBaseUri.size());

  std::size_t uriSeparator = path.find("/");

  auto dataset = path.substr(0, uriSeparator);

  if (uriSeparator != std::string::npos)
    path = path.substr(uriSeparator);
  else
    path = "";

  return std::make_pair(dataset, path);
}

//==============================================================================
std::string OnlineDatabase::getManifestImpl(const std::string& uri)
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
void OnlineDatabase::updateDatasetMap(bool fetchImmediately)
{
  std::unique_lock<std::mutex> lock(mDatasetMapMutex);

  // Continually update the dataset map when requested.
  while (!mStop)
  {
    if (fetchImmediately)
      fetchImmediately = false;
    else // Wait for an update request.
      mUpdateCacheCondition.wait(lock);

    // Exit if notified and stopped.
    if (mStop)
      break;

    // Update the model cache.
    if (updateDatasetMapImpl())
    {
      std::lock_guard<std::mutex> lock2(mCallbacksMutex);
      DART_UNUSED(lock2);

      if (mStop)
        break;

      //      modelDBUpdated(modelCache); // TODO: Change to signal
    }
    else
    {
      dterr << "Unable to download model manifests.\n";
    }

    mUpdateDatasetMapCompleteCondition.notify_all();
  }

  // Make sure no one is waiting on us.
  mUpdateDatasetMapCompleteCondition.notify_all();
}

//==============================================================================
bool OnlineDatabase::updateDatasetMapImpl()
{
  const std::string xmlString = getDatabaseConfig(getServerUri());

  if (xmlString.empty())
    return true;

  tinyxml2::XMLDocument xmlDoc;
  xmlDoc.Parse(xmlString.c_str());

  const auto* databaseElem = xmlDoc.FirstChildElement("database");
  if (!databaseElem)
  {
    dterr << "No <database> tag in the model database "
          << mDatabaseConfigFilename << " found"
          << " here[" << getServerUri() << "].\n";
    return false;
  }

  const auto* modelsElem = databaseElem->FirstChildElement("models");
  if (!modelsElem)
  {
    dterr << "No <models> tag in the model database " << mDatabaseConfigFilename
          << " found"
          << " here[" << getServerUri() << "].\n";
    return false;
  }

  for (auto* uriElem = modelsElem->FirstChildElement("uri");
       uriElem != nullptr && !mStop;
       uriElem = uriElem->NextSiblingElement("uri"))
  {
    const std::string uri = uriElem->GetText();

    const size_t index = uri.find("://");
    std::string suffix = uri;
    if (index != std::string::npos)
      suffix = uri.substr(index + 3, uri.size() - index - 3);

    const std::string fullUri = getServerUri() + suffix;
    const std::string modelName = getDatasetName(fullUri);

    mDatasetMap[fullUri] = modelName;
  }

  return true;
}

} // namespace utils
} // namespace dart

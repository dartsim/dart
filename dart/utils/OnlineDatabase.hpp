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

#ifndef DART_UTILS_ONLINEDATABASE_HPP_
#define DART_UTILS_ONLINEDATABASE_HPP_

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Singleton.hpp"
#include "dart/common/Uri.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace dart {
namespace utils {

class OnlineDatabase final
{
public:
  /// Constructor.
  OnlineDatabase(
      const std::string& serverUri,
      const std::string& baseUri,
      const std::string& databaseLocalPath,
      const std::string& databaseConfigFilename = "database.config",
      const std::string& datasetConfigFilename = "skeleton.config");

  /// Destructor.
  virtual ~OnlineDatabase();

  /// Starts the online database.
  ///
  /// \param[in] fetchImmediately True to fetch the datasets without waiting.
  void startup(bool fetchImmediately = false);

  /// Finalizes the online database.
  void shutdown();

  /// Returns the the global online database URI.
  ///
  /// \return The URI.
  std::string getServerUri();

  /// Returns the dictionary of all the dataset names.
  ///
  /// This is a blocking call, which means it will wait for the OnlineDatabase
  /// to download the dataset list.
  /// \return A map of dataset names, indexed by their full URI.
  const std::map<std::string, std::string>& getDatasets();

  /// Returns the dictionary of all dataset names via a callback.
  ///
  /// This is the non-blocking version of getDatasets
  /// \param[in] func Callback function that receives the list of models.
  /// \return A boost shared pointer. This pointer must remain valid in order to
  /// receive the callback.
  //  event::ConnectionPtr GetModels(
  //      std::function<void(const std::map<std::string, std::string>&)> func);

  /// Returns the name of a dataset based on a URI.
  ///
  /// The URI must be fully qualified:
  /// http://gazebosim.org/gazebo_models/ground_plane or dart://gazebo_models
  /// \param[in] uri The dataset uri
  /// \return The dataset's name.
  std::string getDatasetName(const std::string& uri);

  /// Return the dataset.config file as a XML string.
  ///
  /// \return The dataset config file from the online database.
  std::string getDatasetConfig(const std::string& datasetUri);

  /// Return the database.config file as a XML string.
  ///
  /// \return The database config file from the online database.
  std::string getDatabaseConfig(const std::string& configUri);

  /// Returns the local path to a dataset.
  ///
  /// Returns the path to a model based on a URI. If the model is on a remote
  /// server, then the model fetched and installed locally.
  /// \param[in] uri The model uri
  /// \param[in] forceDownload True to skip searching local paths.
  /// \return Path to a model directory
  std::string getDatasetPath(std::string uri, bool forceDownload = false);

  /// Returns a dataset's file based on a URI.
  ///
  /// Returns a model file based on a URI. If the model is on a remote server,
  /// then the model fetched and installed locally.
  /// \param[in] uri The URI of the model
  /// \return The full path and filename to the SDF file
  std::string getDataPath(const std::string& uri);

  /// Downloads all dependencies for a give model path
  ///
  /// Look's in the model's manifest file (path/model.config) for all models
  /// listed in the <depend> block, and downloads the models if necessary.
  /// \param[in] path Path to a model.
  void downloadDependencies(const std::string& path);

  /// Returns true if the dataset exists on the database.
  ///
  /// \param[in] modelUri URI of the model (eg: model://my_model_name).
  /// \return True if the model was found.
  bool hasDataset(const std::string& datasetUri);

private:
  std::pair<std::string, std::string> resolveDatasetAndData(
      const std::string& uri);

  /// A helper function that uses CURL to get a manifest file.
  ///
  /// \param[in] uri URI of a manifest XML file.
  /// \return The contents of the manifest file.
  std::string getManifestImpl(const std::string& uri);

  /// Updates the dataset cache.
  ///
  /// This function is called by \c updateCacheThread.
  /// \param[in] fetchImmediately True to fetch the models without waiting.
  void updateDatasetMap(bool fetchImmediately);

  /// Used by updateModelCache, no one else should use this function.
  bool updateDatasetMapImpl();

  const std::string mServerUri;
  const std::string mBaseUri;
  const std::string mDatabaseLocalpath;
  const std::string mDatabaseConfigFilename;
  const std::string mDatasetConfigFilename;

  /// Thread to update the dataset cache.
  std::thread* mUpdateDatasetMapThread;

  /// A dictionary of all model names indexed by their uri.
  std::map<std::string, std::string> mDatasetMap;

  /// True to stop the background thread.
  bool mStop;

  /// Dataset map update mutex.
  std::mutex mDatasetMapMutex;

  /// Protects callback list.
  std::mutex mCallbacksMutex;

  /// Mutex to protect cache thread status checks.
  std::recursive_mutex mUpdateDatasetThreadMutex;

  /// Condition variable for the updateCacheThread.
  std::condition_variable mUpdateCacheCondition;

  /// Condition variable for completion of one cache update.
  std::condition_variable mUpdateDatasetMapCompleteCondition;

  /// \def CallbackFunc
  /// Boost function that is used to passback the model cache.
  using CallbackFunc
      = std::function<void(const std::map<std::string, std::string>&)>;

  /// Triggered when the model data has been updated after
  /// calling ModelDatabase::GetModels()
  //  event::EventT<void(std::map<std::string, std::string>)> modelDBUpdated;
};

} // namespace utils
} // namespace dart

#endif // DART_UTILS_ONLINEDATABASE_HPP_

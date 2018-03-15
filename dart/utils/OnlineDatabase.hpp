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

class OnlineDatabase final : public common::Singleton<OnlineDatabase>
{
public:
  /// Starts the model database.
  /// \param[in] _fetchImmediately True to fetch the models without
  /// waiting.
  void startup(bool fetchImmediately = false);

  /// Finalizes the model database.
  void shutdown();

  /// Returns the the global model database URI.
  /// \return the URI.
  std::string getUri();

  /// Returns the dictionary of all the model names
  ///
  /// This is a blocking call. Which means it will wait for the
  /// ModelDatabase to download the model list.
  /// \return a map of model names, indexed by their full URI.
  std::map<std::string, std::string> getModels();

  /// Returns the dictionary of all model names via a callback.
  ///
  /// This is the non-blocking version of ModelDatabase::GetModels
  /// \param[in] _func Callback function that receives the list of
  /// models.
  /// \return A boost shared pointer. This pointer must remain valid in
  /// order to receive the callback.
  //  event::ConnectionPtr GetModels(
  //      std::function<void(const std::map<std::string, std::string>&)> func);

  /// Returns the name of a model based on a URI.
  ///
  /// The URI must be fully qualified:
  /// http://gazebosim.org/gazebo_models/ground_plane or model://gazebo_models
  /// \param[in] uri The model uri
  /// \return The model's name.
  std::string GetModelName(const std::string& uri);

  /// Return the model.config file as a string.
  /// \return The model config file from the model database.
  std::string GetModelConfig(const std::string& modelUri);

  /// Return the database.config file as a string.
  /// \return The database config file from the model database.
  std::string GetDBConfig(const std::string& configUri);

  /// Returns the local path to a model.
  ///
  /// Get the path to a model based on a URI. If the model is on a remote
  /// server, then the model fetched and installed locally.
  /// \param[in] uri The model uri
  /// \param[in] forceDownload True to skip searching local paths.
  /// \return path to a model directory
  std::string GetModelPath(const std::string& uri, bool forceDownload = false);

  /// Returns a model's SDF file based on a URI.
  ///
  /// Get a model file based on a URI. If the model is on
  /// a remote server, then the model fetched and installed locally.
  /// \param[in] uri The URI of the model
  /// \return The full path and filename to the SDF file
  std::string GetModelFile(const std::string& uri);

  /// Downloads all dependencies for a give model path
  ///
  /// Look's in the model's manifest file (_path/model.config)
  /// for all models listed in the <depend> block, and downloads the
  /// models if necessary.
  /// \param[in] _path Path to a model.
  void DownloadDependencies(const std::string& path);

  /// Returns true if the model exists on the database.
  ///
  /// \param[in] modelName URI of the model (eg: model://my_model_name).
  /// \return True if the model was found.
  bool HasModel(const std::string& modelName);
  bool hasData(const common::Uri& dataUri);

protected:
  friend class common::Singleton<OnlineDatabase>;

  OnlineDatabase();

  virtual ~OnlineDatabase();

  /// A helper function that uses CURL to get a manifest file.
  /// \param[in] uri URI of a manifest XML file.
  /// \return The contents of the manifest file.
  std::string GetManifestImpl(const std::string& uri);

  /// Updates the model cache.
  ///
  /// This function is called by \c updateCacheThread.
  /// \param[in] fetchImmediately True to fetch the models without waiting.
  void UpdateModelCache(bool fetchImmediately);

  /// Used by ModelDatabase::UpdateModelCache,
  /// no one else should use this function.
  bool UpdateModelCacheImpl();

  /// Thread to update the model cache.
  std::thread* updateCacheThread;

  /// A dictionary of all model names indexed by their uri.
  std::map<std::string, std::string> modelCache;

  /// True to stop the background thread
  bool stop;

  /// Cache update mutex.
  std::mutex updateMutex;

  /// Protects callback list.
  std::mutex callbacksMutex;

  /// Mutex to protect cache thread status checks.
  std::recursive_mutex startCacheMutex;

  /// Condition variable for the updateCacheThread.
  std::condition_variable updateCacheCondition;

  /// Condition variable for completion of one cache update.
  std::condition_variable updateCacheCompleteCondition;

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

// Project addition, 2026-10-05. Distributed with FAST-LIVO2 under GPL-2.0.
#pragma once
#include <opencv2/core.hpp>
#include <cstdint>
#include <memory>
#include <string>

namespace livo_memory {
struct ImageLimits {
  size_t hot_bytes = 256ULL << 20, cold_bytes = 8ULL << 30;
  size_t records = 10000, files = 10000, jobs = 8, job_bytes = 64ULL << 20;
  size_t image_bytes = 32ULL << 20;
};
struct ImageStats {
  size_t records = 0, hot_bytes = 0, cold_bytes = 0, files = 0;
  size_t jobs = 0, job_bytes = 0, leases = 0, hot_images = 0;
  uint64_t writes = 0, loads = 0, hits = 0, failures = 0, rejected = 0;
};
class ImageState;
class ImageToken {
public:
  ~ImageToken();
  uint64_t id() const { return id_; }
  ImageToken(const ImageToken&) = delete;
  ImageToken& operator=(const ImageToken&) = delete;
private:
  friend class ImageState;
  ImageToken(std::shared_ptr<ImageState> state, uint64_t id);
  std::shared_ptr<ImageState> state_;
  uint64_t id_;
};
using ImageHandle = std::shared_ptr<ImageToken>;
// The custom deleter ends the lease and runs deferred collection.
using ImageLease = std::shared_ptr<const cv::Mat>;
class ImageStore {
public:
  ImageStore(const std::string& parent, const ImageLimits& limits);
  ~ImageStore();
  ImageStore(const ImageStore&) = delete;
  ImageStore& operator=(const ImageStore&) = delete;
  ImageHandle insert(const cv::Mat& image);
  ImageLease acquire(const ImageHandle& image);
  void cool();
  void waitIdle();
  void collect();
  void shutdown();
  ImageStats stats() const;
  std::string directory() const;
  std::string error() const;
private:
  std::shared_ptr<ImageState> state_;
};
}

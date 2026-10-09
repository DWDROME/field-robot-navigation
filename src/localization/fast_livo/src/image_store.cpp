// Project addition, 2026-10-05. Distributed with FAST-LIVO2 under GPL-2.0.
#include "image_store.h"
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <sstream>
#include <mutex>
#include <sys/stat.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>

namespace livo_memory {
namespace {
struct File {
  int fd;
  explicit File(int value) : fd(value) {}
  ~File() { if (fd >= 0) ::close(fd); }
  bool close() { int value = fd; fd = -1; return ::close(value) == 0; }
};
bool transfer(int fd, void* data, size_t bytes, bool write) {
  auto* p = static_cast<unsigned char*>(data);
  while (bytes) {
    ssize_t n = write ? ::write(fd, p, bytes) : ::read(fd, p, bytes);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) return false;
    bytes -= static_cast<size_t>(n); p += n;
  }
  return true;
}
uint64_t checksum(const cv::Mat& image) {
  uint64_t hash = 14695981039346656037ULL;
  for (size_t i = 0; i < image.total() * image.elemSize(); ++i)
    hash = (hash ^ image.data[i]) * 1099511628211ULL;
  return hash;
}
struct Header { uint64_t magic, id, rows, cols, type, bytes, hash; };
constexpr uint64_t magic = 0x324f56494c545346ULL;
}
class ImageState : public std::enable_shared_from_this<ImageState> {
public:
  struct Record {
    std::weak_ptr<ImageToken> token;
    std::shared_ptr<cv::Mat> pixels;
    Header header{};
    uint64_t touched = 0;
    size_t leases = 0;
    bool writing = false, loading = false, cold = false, failed = false;
    bool owned_file = false;
    dev_t device = 0;
    ino_t inode = 0;
  };
  mutable std::mutex mutex;
  std::condition_variable wake;
  ImageLimits limits;
  ImageStats counts;
  std::unordered_map<uint64_t, Record> records;
  std::deque<uint64_t> queue;
  std::thread worker;
  std::mutex join_mutex;
  std::string path, last_error;
  int dirfd = -1;
  dev_t manifest_device = 0;
  ino_t manifest_inode = 0;
  uint64_t next_id = 1, clock = 0;
  bool stopping = false;

  ImageState(const std::string& parent, const ImageLimits& value) : limits(value) {
    if (!limits.hot_bytes || !limits.cold_bytes || !limits.records || !limits.files ||
        !limits.jobs || !limits.job_bytes || !limits.image_bytes ||
        limits.image_bytes > limits.hot_bytes || limits.image_bytes > limits.job_bytes)
      throw std::invalid_argument("FAST-LIVO2 invalid image budgets");
    namespace fs = std::filesystem;
    fs::path root = fs::canonical(parent);
#ifdef ROOT_DIR
    auto source = fs::weakly_canonical(ROOT_DIR).lexically_normal();
    auto relative = root.lexically_relative(source);
    if (relative.empty() || *relative.begin() != "..")
      throw std::invalid_argument("FAST-LIVO2 cold directory must be outside source");
#endif
    std::string pattern = (root / "fastlivo2-XXXXXX").string();
    if (!::mkdtemp(pattern.data())) throw std::runtime_error("FAST-LIVO2 cannot create cold run directory");
    path = pattern;
    dirfd = ::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (dirfd < 0) throw std::runtime_error("FAST-LIVO2 cannot open cold run directory");
    if (!manifestLocked()) { ::close(dirfd); dirfd = -1; throw std::runtime_error(last_error); }
    try { worker = std::thread([this] { work(); }); }
    catch (...) { ::close(dirfd); dirfd = -1; throw; }
  }
  ~ImageState() { stop(); if (dirfd >= 0) ::close(dirfd); }
  std::string filename(uint64_t id) const { return std::to_string(id) + ".livo"; }
  // Bounded snapshot, replaced atomically. No append-only history.
  bool manifestLocked() {
    struct stat previous{};
    if (::fstatat(dirfd, "owned.txt", &previous, AT_SYMLINK_NOFOLLOW) == 0) {
      if (!manifest_inode || previous.st_dev != manifest_device || previous.st_ino != manifest_inode || !S_ISREG(previous.st_mode)) {
        failure(0, "manifest path replaced; preserving foreign entry"); return false;
      }
    } else if (errno != ENOENT) { failure(0, "cannot inspect manifest"); return false; }
    std::ostringstream contents;
    contents << "FAST-LIVO2 owned files v1\n";
    for (const auto& item : records) if (item.second.owned_file)
      contents << filename(item.first) << ' ' << item.second.device << ' ' << item.second.inode << '\n';
    auto bytes = contents.str();
    File file(::openat(dirfd, "owned.next", O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600));
    if (file.fd < 0) { failure(0, "cannot create owned manifest"); return false; }
    bool ok = transfer(file.fd, bytes.data(), bytes.size(), true) && ::fsync(file.fd) == 0;
    struct stat identity{};
    ok = ::fstat(file.fd, &identity) == 0 && ok;
    ok = file.close() && ok;
    if (ok) {
      ok = ::renameat(dirfd, "owned.next", dirfd, "owned.txt") == 0;
      if (ok) { manifest_device = identity.st_dev; manifest_inode = identity.st_ino; ok = ::fsync(dirfd) == 0; }
    }
    if (!ok) { ::unlinkat(dirfd, "owned.next", 0); failure(0, "cannot persist owned manifest"); }
    return ok;
  }
  void failure(uint64_t id, const std::string& message) {
    ++counts.failures;
    last_error = "image=" + std::to_string(id) + " directory=" + path + " " + message;
    // State changes only, not a per-frame wait loop.
    std::cerr << "[ FAST-LIVO2 memory ] " << last_error << '\n';
  }
  bool removeFile(uint64_t id) {
    auto name = filename(id);
    struct stat info{};
    if (::fstatat(dirfd, name.c_str(), &info, AT_SYMLINK_NOFOLLOW) != 0) {
      if (errno == ENOENT) return true;
      failure(id, "cannot inspect owned file"); return false;
    }
    const auto& record = records.at(id);
    if (!S_ISREG(info.st_mode) || info.st_dev != record.device || info.st_ino != record.inode) {
      failure(id, "owned path replaced; preserving foreign entry"); return false;
    }
    if (::unlinkat(dirfd, name.c_str(), 0) != 0) { failure(id, "cannot remove owned file"); return false; }
    return true;
  }
  void collectLocked() {
    for (auto it = records.begin(); it != records.end();) {
      auto& r = it->second;
      if (!r.token.expired() || r.leases || r.writing || r.loading) { ++it; continue; }
      if (r.owned_file && !removeFile(it->first)) { ++it; continue; }
      if (r.owned_file) { r.owned_file = false; --counts.files; counts.cold_bytes -= sizeof(Header) + r.header.bytes; manifestLocked(); }
      if (r.pixels) counts.hot_bytes -= r.header.bytes;
      it = records.erase(it);
    }
  }
  bool roomLocked(size_t bytes) {
    if (bytes > limits.hot_bytes) return false;
    while (counts.hot_bytes > limits.hot_bytes - bytes) {
      Record* oldest = nullptr;
      for (auto& item : records) {
        auto& r = item.second;
        if (r.pixels && r.cold && !r.leases && !r.writing && !r.loading &&
            (!oldest || r.touched < oldest->touched)) oldest = &r;
      }
      if (!oldest) return false;
      counts.hot_bytes -= oldest->header.bytes;
      oldest->pixels.reset();
    }
    return true;
  }
  ImageHandle insert(const cv::Mat& image) {
    std::lock_guard<std::mutex> lock(mutex);
    collectLocked();
    const size_t bytes = image.total() * image.elemSize();
    if (stopping || image.empty() || image.type() != CV_8UC1 || bytes > limits.image_bytes ||
        records.size() >= limits.records || !roomLocked(bytes)) { ++counts.rejected; return {}; }
    const auto id = next_id++;
    // Declare before the lock guard is destroyed on failure: an inactive token
    // cannot recursively collect while this mutex is held.
    auto token = ImageHandle(new ImageToken({}, id));
    Record r;
    // One owned, immutable allocation; input/Frame may continue to mutate their own buffer.
    r.pixels = std::make_shared<cv::Mat>(image.clone());
    r.header = {magic, id, static_cast<uint64_t>(image.rows), static_cast<uint64_t>(image.cols),
                static_cast<uint64_t>(image.type()), bytes, checksum(*r.pixels)};
    r.token = token; r.touched = ++clock;
    records.emplace(id, std::move(r)); counts.hot_bytes += bytes;
    token->state_ = shared_from_this();
    return token;
  }
  ImageLease acquire(const ImageHandle& token) {
    if (!token || token->state_.get() != this) return {};
    std::unique_lock<std::mutex> lock(mutex);
    auto id = token->id();
    auto found = records.find(id);
    if (found == records.end()) return {};
    wake.wait(lock, [&] { return !records.at(id).loading; });
    auto& r = records.at(id);
    if (!r.pixels) {
      if (stopping || !r.cold || r.failed || !roomLocked(r.header.bytes)) { ++counts.rejected; return {}; }
      r.loading = true; counts.hot_bytes += r.header.bytes;
      const auto expected = r.header;
      lock.unlock();
      std::shared_ptr<cv::Mat> loaded;
      bool ok = false;
      try {
        File file(::openat(dirfd, filename(id).c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC));
        struct stat info{}; Header actual{};
        if (file.fd >= 0 && ::fstat(file.fd, &info) == 0 && S_ISREG(info.st_mode) &&
            static_cast<uint64_t>(info.st_size) == sizeof(Header) + expected.bytes &&
            transfer(file.fd, &actual, sizeof(actual), false) &&
            actual.magic == expected.magic && actual.id == expected.id && actual.rows == expected.rows &&
            actual.cols == expected.cols && actual.type == expected.type && actual.bytes == expected.bytes && actual.hash == expected.hash) {
          loaded = std::make_shared<cv::Mat>(static_cast<int>(expected.rows), static_cast<int>(expected.cols), CV_8UC1);
          ok = transfer(file.fd, loaded->data, expected.bytes, false) && checksum(*loaded) == expected.hash;
        }
      } catch (const std::exception&) { ok = false; }
      lock.lock();
      // unordered_map can rehash while I/O runs; obtain the record again.
      auto& completed = records.at(id);
      completed.loading = false;
      if (!ok) {
        counts.hot_bytes -= expected.bytes; completed.failed = true;
        failure(id, "cold read missing, damaged or failed"); wake.notify_all(); return {};
      }
      completed.pixels = std::move(loaded); ++counts.loads; wake.notify_all();
    } else ++counts.hits;
    auto& ready = records.at(id);
    ready.touched = ++clock; ++ready.leases; ++counts.leases;
    auto pixels = ready.pixels;
    const auto* pointer = pixels.get();
    auto state = shared_from_this();
    return ImageLease(pointer, [state, id, pixels = std::move(pixels)](const cv::Mat*) mutable {
      pixels.reset();
      std::lock_guard<std::mutex> guard(state->mutex);
      --state->records.at(id).leases; --state->counts.leases;
      state->collectLocked(); state->wake.notify_all();
    });
  }
  void cool() {
    std::lock_guard<std::mutex> lock(mutex);
    collectLocked();
    if (stopping) return;
    for (auto& item : records) {
      auto& r = item.second;
      if (!r.pixels || r.writing || r.loading || r.failed) continue;
      if (r.cold) {
        if (!r.leases) { r.pixels.reset(); counts.hot_bytes -= r.header.bytes; }
        continue;
      }
      const auto disk_bytes = sizeof(Header) + r.header.bytes;
      if (counts.jobs >= limits.jobs || r.header.bytes > limits.job_bytes - counts.job_bytes ||
          counts.files >= limits.files || disk_bytes > limits.cold_bytes - counts.cold_bytes) {
        ++counts.rejected; continue;
      }
      // Reserve before scheduling; partially written files count until removed.
      queue.push_back(item.first);
      r.writing = true;
      ++counts.jobs; counts.job_bytes += r.header.bytes;
      ++counts.files; counts.cold_bytes += disk_bytes;
    }
    wake.notify_all();
  }
  void work() {
    std::unique_lock<std::mutex> lock(mutex);
    for (;;) {
      wake.wait(lock, [&] { return stopping || !queue.empty(); });
      if (queue.empty() && stopping) return;
      auto id = queue.front(); queue.pop_front();
      auto pixels = records.at(id).pixels;
      auto header = records.at(id).header;
      lock.unlock();
      File file(::openat(dirfd, filename(id).c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600));
      const bool created = file.fd >= 0;
      struct stat identity{};
      bool identified = created && ::fstat(file.fd, &identity) == 0;
      lock.lock();
      auto& pending = records.at(id);
      pending.owned_file = created;
      pending.device = identity.st_dev; pending.inode = identity.st_ino;
      bool registered = identified && manifestLocked();
      lock.unlock();
      bool ok = registered && transfer(file.fd, &header, sizeof(header), true) &&
        transfer(file.fd, pixels->data, header.bytes, true) && ::fsync(file.fd) == 0;
      if (file.fd >= 0) ok = file.close() && ok;
      pixels.reset();
      lock.lock();
      auto& r = records.at(id);
      r.writing = false; --counts.jobs; counts.job_bytes -= header.bytes;
      if (ok) {
        r.cold = true; ++counts.writes;
        if (!r.leases) { r.pixels.reset(); counts.hot_bytes -= header.bytes; }
      } else {
        r.failed = true; failure(id, "cold write failed; retaining hot image");
        if (!created || removeFile(id)) {
          r.owned_file = false; --counts.files; counts.cold_bytes -= sizeof(Header) + header.bytes;
          manifestLocked();
        }
      }
      collectLocked(); wake.notify_all();
    }
  }
  void stop() {
    std::lock_guard<std::mutex> join(join_mutex);
    { std::lock_guard<std::mutex> lock(mutex); stopping = true; wake.notify_all(); }
    if (worker.joinable()) worker.join();
    std::unique_lock<std::mutex> lock(mutex);
    wake.wait(lock, [&] { for (const auto& item : records) if (item.second.loading) return false; return true; });
    collectLocked();
  }
};
ImageToken::ImageToken(std::shared_ptr<ImageState> state, uint64_t id) : state_(std::move(state)), id_(id) {}
ImageToken::~ImageToken() { if (state_) { std::lock_guard<std::mutex> lock(state_->mutex); state_->collectLocked(); } }
ImageStore::ImageStore(const std::string& parent, const ImageLimits& limits) : state_(std::make_shared<ImageState>(parent, limits)) {}
ImageStore::~ImageStore() { shutdown(); }
ImageHandle ImageStore::insert(const cv::Mat& image) { return state_->insert(image); }
ImageLease ImageStore::acquire(const ImageHandle& image) { return state_->acquire(image); }
void ImageStore::cool() { state_->cool(); }
void ImageStore::waitIdle() { std::unique_lock<std::mutex> lock(state_->mutex); state_->wake.wait(lock, [&] { return state_->counts.jobs == 0; }); }
void ImageStore::collect() { std::lock_guard<std::mutex> lock(state_->mutex); state_->collectLocked(); }
void ImageStore::shutdown() { if (state_) state_->stop(); }
ImageStats ImageStore::stats() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  auto result = state_->counts; result.records = state_->records.size();
  for (const auto& item : state_->records) if (item.second.pixels) ++result.hot_images;
  return result;
}
std::string ImageStore::directory() const { return state_->path; }
std::string ImageStore::error() const { std::lock_guard<std::mutex> lock(state_->mutex); return state_->last_error; }
}

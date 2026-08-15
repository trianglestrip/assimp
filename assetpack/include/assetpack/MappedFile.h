#pragma once
// ============================================================
// MappedFile - memory-mapped file (Windows)
//
// The physical basis of the zero-copy chain: the file contents
// are never copied anywhere. All later views (JSON DOM strings,
// GLB BIN chunk, bufferView geometry data) point directly into
// this mapping, which is kept alive via shared_ptr ownership.
// ============================================================

#include <cstdint>
#include <memory>
#include <span>
#include <string_view>

namespace ap {

class MappedFile {
public:
    MappedFile() = default;
    MappedFile(const MappedFile&) = delete;
    MappedFile& operator=(const MappedFile&) = delete;
    ~MappedFile();

    // Open and map `path` read-only. Returns false on any failure
    // (file missing, empty file, mapping denied).
    bool open(std::string_view path);

    // View over the whole file. Empty if not open.
    std::span<const std::byte> bytes() const { return bytes_; }

    bool isOpen() const { return bytes_.data() != nullptr; }
    size_t size() const { return bytes_.size(); }

    // Convenience: reinterpret the mapping as chars (JSON chunk etc.)
    std::string_view text() const {
        return { reinterpret_cast<const char*>(bytes_.data()), bytes_.size() };
    }

    // Shared ownership so views can outlive the loader object.
    static std::shared_ptr<MappedFile> openShared(std::string_view path);

private:
    void close();

    void*  file_       = nullptr;  // HANDLE
    void*  mapping_    = nullptr;  // HANDLE
    void*  base_       = nullptr;  // MapViewOfFile base
    std::span<const std::byte> bytes_;
};

} // namespace ag

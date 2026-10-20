#ifndef DELTA_BASIC_PATH_H
#define DELTA_BASIC_PATH_H

#include <string>

#include <filesystem>

namespace dbasic {

    namespace filesystem = std::filesystem;

    class Path {
    public:
        Path(const filesystem::path &path);
        Path(const std::string &path);
        Path(const char *path);
        Path(const Path &path);
        Path();
        ~Path();

        std::string ToString() const;

        void SetPath(const std::string &path);
        bool operator==(const Path &path) const;
        Path Append(const Path &path) const;

        void GetParentPath(Path *path) const;

        const Path &operator=(const Path &b);

        std::string GetExtension() const;
        std::string GetStem() const;

        Path GetAbsolute() const;

        bool IsAbsolute() const;
        bool Exists() const;

        bool CreateDir() const;

    protected:
        filesystem::path *m_path;

    protected:
        const filesystem::path &GetPath() const { return *m_path; }
    };

} /* namespace dbasic */

#endif /* DELTA_BASIC_PATH_H */
